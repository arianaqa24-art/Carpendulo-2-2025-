#include <Arduino.h>
#include <math.h>
#include <Encoder.h>
#include <Ultrasonic.h>
#ifdef SP
  #undef SP
#endif

//  GANANCIAS LQR 
// u = -k_theta*theta - k_omega*omega - k_x*x_riel
float k_theta = -2131.24141086597f;   // ganancia sobre el error de posición (grados)
float k_omega = -713.706599826982f;   // ganancia sobre la velocidad (deg/s)
float k_x     = -561.206758612675f;    // ganancia distancia 

//  HARDWARE 
const uint8_t PIN_TRIG        = 5;
const uint8_t PIN_ECHO        = 6;
const uint8_t PIN_ENAB        = 9;
const uint8_t PIN_HORARIO     = 4;
const uint8_t PIN_ANTIHORARIO = 7;
const uint8_t PIN_ENC_A       = 18;
const uint8_t PIN_ENC_B       = 19;
const int     PPR             = 20;

Encoder    encoder(PIN_ENC_A, PIN_ENC_B);
Ultrasonic ultra(PIN_TRIG, PIN_ECHO);

// Deadzone para evitar oscilaciones pequeñas ---
const float U_DEAD_VOLT    = 1.2f;  // si |u| < 0.20 V -> considerar 0 (ajústalo)
const float U_RESUME_VOLT  = 1.5f;  // si |u| >= 0.30 V -> salir de deadzone (histeresis)
static bool  in_deadzone   = false;  // estado para la histeresis


// Encoder: cuentas → grados
const float QUAD_X         = 4.0f;
const float COUNTS_PER_REV = PPR * QUAD_X;
const float DEG_PER_COUNT  = 360.0f / COUNTS_PER_REV;
static inline float countsToDeg(float counts) { return counts * DEG_PER_COUNT; }

static inline uint8_t voltsA_PWM(float v) {
  if (v < 0) v = 0;
  if (v > 5) v = 5;
  return (uint8_t)lroundf((v / 5.0f) * 255.0f);
}

static inline float leerDistanciaM() {
  float cm = ultra.read();
  if (cm <= 0) return 0;
  return cm;
}

//  INTERPOLACIÓN u_sinin -> voltaje firmado (-5..+5) 
const float V_MAX = 5.0f;                 // Volts máximos del actuador
const float U_SYM_MAX_VIN = 500000.0f;       // <-- AJUSTA este valor según tus pruebas/registro

static inline float mapSignedVolt(float u_sinin) {
  // Evitar division por cero
  float denom = (U_SYM_MAX_VIN == 0.0f) ? 1.0f : U_SYM_MAX_VIN;
  float v_signed = (u_sinin / denom) * V_MAX;
  // Saturación
  if (v_signed > V_MAX) v_signed = V_MAX;
  if (v_signed < -V_MAX) v_signed = -V_MAX;
  return v_signed;
}

// Referencia de posición en la riel
const float X_REF_CM = 15.0f;

// moditos
enum Mode { MODE_CONTROL = 0, MODE_HOMING = 1, MODE_HOLD = 2 };
Mode mode = MODE_CONTROL;

// Parámetros de homing (del LQR)
const float homing_pwm_volts    = 2.0f;
const float homing_tol_cm       = 0.6f;
const unsigned long hold_duration_ms = 400;

// Búsqueda cuando ultrasónica no responde
const unsigned long search_max_ms = 2000;
unsigned long search_start_ms = 0;
bool  search_active = false;
unsigned long hold_start_ms = 0;

// KALMAN: dimensiones y matrices 
const int N  = 3;
const int Pm = 3;

float A[N][N] = {
  { 1.1315f, -0.0274f,  0.2064f},
  { 0.0211f,  0.8942f, -0.4322f},
  {-0.0851f,  0.3884f,  0.8818f}
};

float B[N] = {
  -0.00008486f,
   0.00002181f,
  -0.00009874f
};

float C[Pm][N] = {
  {1.0f, 0.0f, 0.0f},
  {0.0f, 1.0f, 0.0f},
  {0.0f, 0.0f, 1.0f}
};

float Q[N][N] = {
  {1.0e-1f, 0.0f,    0.0f   },
  {0.0f,    1.0e-1f, 0.0f   },
  {0.0f,    0.0f,    1.0f}
};

float R[Pm][Pm] = {
  {1.0e-10f, 0.0f,    0.0f   },
  {0.0f,    1.0e-10f, 0.0f   },
  {0.0f,    0.0f,    1.0e-10f}
};

float I3[N][N] = {
  {1.0f, 0.0f, 0.0f},
  {0.0f, 1.0f, 0.0f},
  {0.0f, 0.0f, 1.0f}
};

float x_est[N]        = {0.0f, 0.0f, 0.0f};
float x_est_priori[N] = {0.0f, 0.0f, 0.0f};

float P_est[N][N] = {
  {0.1f, 0.0f, 0.0f},
  {0.0f, 0.1f, 0.0f},
  {0.0f, 0.0f, 0.1f}
};
float P_est_priori[N][N];

float K_kal[N][Pm];
float S[Pm][Pm];
float S_inv[Pm][Pm];

float u_model_prev = 0.0f;

//  Helpers de matrices 3x3 
void mat3_mul(float A_[3][3], float B_[3][3], float R_[3][3]) {
  for (int i=0; i<3; i++) {
    for (int j=0; j<3; j++) {
      float s = 0.0f;
      for (int k=0; k<3; k++) s += A_[i][k] * B_[k][j];
      R_[i][j] = s;
    }
  }
}

void mat3_mul_vec(float A_[3][3], float v_[3], float r_[3]) {
  for (int i=0; i<3; i++) {
    float s = 0.0f;
    for (int k=0; k<3; k++) s += A_[i][k] * v_[k];
    r_[i] = s;
  }
}

void mat3_transpose(float A_[3][3], float AT_[3][3]) {
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      AT_[j][i] = A_[i][j];
}

bool mat3_inv(float M[3][3], float Minv[3][3]) {
  float det =
      M[0][0]*(M[1][1]*M[2][2] - M[1][2]*M[2][1])
    - M[0][1]*(M[1][0]*M[2][2] - M[1][2]*M[2][0])
    + M[0][2]*(M[1][0]*M[2][1] - M[1][1]*M[2][0]);

  if (fabs(det) < 1e-9f) return false;
  float invDet = 1.0f / det;

  Minv[0][0] =  (M[1][1]*M[2][2] - M[1][2]*M[2][1]) * invDet;
  Minv[0][1] = -(M[0][1]*M[2][2] - M[0][2]*M[2][1]) * invDet;
  Minv[0][2] =  (M[0][1]*M[1][2] - M[0][2]*M[1][1]) * invDet;

  Minv[1][0] = -(M[1][0]*M[2][2] - M[1][2]*M[2][0]) * invDet;
  Minv[1][1] =  (M[0][0]*M[2][2] - M[0][2]*M[2][0]) * invDet;
  Minv[1][2] = -(M[0][0]*M[1][2] - M[0][2]*M[1][0]) * invDet;

  Minv[2][0] =  (M[1][0]*M[2][1] - M[1][1]*M[2][0]) * invDet;
  Minv[2][1] = -(M[0][0]*M[2][1] - M[0][1]*M[2][0]) * invDet;
  Minv[2][2] =  (M[0][0]*M[1][1] - M[0][1]*M[1][0]) * invDet;

  return true;
}

//  VARIABLES 
unsigned long t_start_us = 0;
static unsigned long t_prev_us   = 0;
static long          counts_prev = 0;

void emitirCSV(float t_s,
               float theta_deg, float omega_deg_s, float x_riel_cm,
               float u_sinin, float u_volt,
               float theta_hat, float omega_hat, float xhat_riel) {
  Serial.print(t_s, 6);        Serial.print(',');
  Serial.print(theta_deg, 6);  Serial.print(',');
  Serial.print(omega_deg_s, 6);Serial.print(',');
  Serial.print(x_riel_cm, 6);  Serial.print(',');
  Serial.print(u_sinin, 6);    Serial.print(',');
  Serial.print(u_volt, 6);     Serial.print(',');
  Serial.print(theta_hat, 6);  Serial.print(',');
  Serial.print(omega_hat, 6);  Serial.print(',');
  Serial.println(xhat_riel, 6);
}

void setup() {
  pinMode(PIN_HORARIO, OUTPUT);
  pinMode(PIN_ANTIHORARIO, OUTPUT);
  pinMode(PIN_ENAB, OUTPUT);

  digitalWrite(PIN_ANTIHORARIO, LOW);
  digitalWrite(PIN_HORARIO, LOW);
  analogWrite(PIN_ENAB, voltsA_PWM(0));

  // modo holding
  float distancia = leerDistanciaM();
  while (isfinite(distancia) &&
         (distancia < (X_REF_CM - 0.5f) || distancia > (X_REF_CM + 0.5f))) {

    if (distancia < X_REF_CM) {
      digitalWrite(PIN_ANTIHORARIO, HIGH);
      digitalWrite(PIN_HORARIO, LOW);
    } else {
      digitalWrite(PIN_ANTIHORARIO, LOW);
      digitalWrite(PIN_HORARIO, HIGH);
    }
    analogWrite(PIN_ENAB, voltsA_PWM(homing_pwm_volts));

    distancia = leerDistanciaM();
  }

  analogWrite(PIN_ENAB, 0);
  digitalWrite(PIN_HORARIO, LOW);
  digitalWrite(PIN_ANTIHORARIO, LOW);

  encoder.write(0L);
  counts_prev = 0;

  Serial.begin(9600);
  while (!Serial) { }

  Serial.println("t_s,theta_deg,omega_deg_s,x_riel_cm,u_sinin,u_volt,theta_hat,omega_hat,xhat_riel");

  t_start_us = micros();
  t_prev_us  = t_start_us;

  float theta0    = countsToDeg((float)encoder.read());
  float omega0    = 0.0f;
  float dist0_cm  = leerDistanciaM();
  float xriel0    = (isfinite(dist0_cm) ? dist0_cm - X_REF_CM : 0.0f);

  x_est[0]        = theta0;
  x_est[1]        = omega0;
  x_est[2]        = xriel0;
  u_model_prev    = 0.0f;
}

void loop() {
  // Cálculo de dt 
  unsigned long t_now_us = micros();
  float dt_s = (t_now_us - t_prev_us) / 1e6f;
  if (dt_s <= 0.0f || dt_s > 0.05f) dt_s = 0.01f;
  t_prev_us = t_now_us;

  // 1) MEDICIONES 
  long counts = encoder.read();
  long delta_counts = counts - counts_prev;
  counts_prev = counts;

  float theta_deg    = countsToDeg((float)counts);
  float omega_deg_s  = (delta_counts * DEG_PER_COUNT) / dt_s;

  float distancia_cm = leerDistanciaM();
  float x_riel_cm    = 0.0f;
  bool  distancia_ok = isfinite(distancia_cm);

  if (distancia_ok) {
    x_riel_cm = distancia_cm - X_REF_CM;
  }

  // Moditos
  bool fuera_zona_segura = !distancia_ok || distancia_cm < 3.5f || distancia_cm > 28.5f;

  if (mode == MODE_CONTROL) {
    if (fuera_zona_segura) {
      mode = MODE_HOMING;
      search_active = false;
      search_start_ms = 0;
    }
  }

  if (mode == MODE_HOMING) {
    if (!distancia_ok) {
      unsigned long now_ms = millis();
      if (!search_active) {
        search_active = true;
        search_start_ms = now_ms;
      }
      unsigned long elapsed = now_ms - search_start_ms;

      if (elapsed <= search_max_ms) {
        digitalWrite(PIN_ANTIHORARIO, HIGH);
        digitalWrite(PIN_HORARIO, LOW);
        analogWrite(PIN_ENAB, voltsA_PWM(homing_pwm_volts));

        float t_s = (micros() - t_start_us) / 1e6f;
        emitirCSV(t_s, theta_deg, omega_deg_s, x_riel_cm, 0.0f, homing_pwm_volts,
                  x_est[0], x_est[1], x_est[2]);
        return;
      } else {
        search_active = false;
        analogWrite(PIN_ENAB, 0);
        digitalWrite(PIN_HORARIO, LOW);
        digitalWrite(PIN_ANTIHORARIO, LOW);
        mode = MODE_HOLD;
        hold_start_ms = millis();
      }
    } else {
      float error_cm = distancia_cm - X_REF_CM;
      if (fabs(error_cm) <= homing_tol_cm) {
        analogWrite(PIN_ENAB, 0);
        digitalWrite(PIN_HORARIO, LOW);
        digitalWrite(PIN_ANTIHORARIO, LOW);
        mode = MODE_HOLD;
        hold_start_ms = millis();
      } else {
        if (distancia_cm > X_REF_CM) {
          digitalWrite(PIN_ANTIHORARIO, LOW);
          digitalWrite(PIN_HORARIO, HIGH);
        } else {
          digitalWrite(PIN_ANTIHORARIO, HIGH);
          digitalWrite(PIN_HORARIO, LOW);
        }
        analogWrite(PIN_ENAB, voltsA_PWM(homing_pwm_volts));

        float t_s = (micros() - t_start_us) / 1e6f;
        emitirCSV(t_s, theta_deg, omega_deg_s, x_riel_cm, 0.0f, homing_pwm_volts,
                  x_est[0], x_est[1], x_est[2]);
        return;
      }
    }
  }

  if (mode == MODE_HOLD) {
    analogWrite(PIN_ENAB, 0);
    digitalWrite(PIN_HORARIO, LOW);
    digitalWrite(PIN_ANTIHORARIO, LOW);

    float t_s = (micros() - t_start_us) / 1e6f;
    emitirCSV(t_s, theta_deg, omega_deg_s, x_riel_cm, 0.0f, 0.0f,
              x_est[0], x_est[1], x_est[2]);

    if ((millis() - hold_start_ms) >= hold_duration_ms) {
      if (distancia_ok && distancia_cm >= 3.5f && distancia_cm <= 28.5f) {
        mode = MODE_CONTROL;
      } else {
        mode = MODE_HOMING;
      }
    } else {
      return;
    }
  }

  // 2) KALMAN (LQG) 

  // Medición actual
  float y_meas[3] = { theta_deg, omega_deg_s, x_riel_cm };

  // PREDICCIÓN
  // Predicción del estado x_est_priori = A*x_est + B*u
  mat3_mul_vec(A, x_est, x_est_priori);  // x_est_priori = A * x_est
  for (int i=0; i<3; i++) {
    x_est_priori[i] += B[i] * u_model_prev;  // + B*u_model_prev
  }

  // Predicción de la covarianza P_est_priori = A*P_est*A' + Q
  float AP[3][3], AT[3][3];
  mat3_mul(A, P_est, AP);       // AP = A * P_est
  mat3_transpose(A, AT);        // AT = A'
  mat3_mul(AP, AT, P_est_priori); // P_est_priori = A*P_est*A'
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      P_est_priori[i][j] += Q[i][j];  // + Q (ruido de proceso)

  // Residuo: diferencia entre medición y predicción
  float innov[3];
  for (int i = 0; i < 3; i++) {
    innov[i] = 0.0f;
    for (int j = 0; j < 3; j++) {
      innov[i] += C[i][j] * x_est_priori[j]; // C * x_est_priori
    }
    innov[i] = y_meas[i] - innov[i];        // y - C*x_est_priori
  }

  // COVARIANZA DE LA INNOVACIÓN
  // S = C*P_est_priori*C' + R
  float CP[3][3], CTP[3][3];
  mat3_mul(C, P_est_priori, CP);     // CP = C * P_est_priori
  mat3_transpose(C, CTP);            // CTP = C'
  mat3_mul(CP, CTP, S);              // S = C*P_est_priori*C'
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      S[i][j] += R[i][j];            // + R (ruido de medición)

  // INVERSA DE S
  if (mat3_inv(S, S_inv)) {  // solo si S es invertible

    // GANANCIA DE KALMAN 
    // K = P_est_priori * C' * S^-1
    float K_temp[3][3];
    mat3_mul(P_est_priori, CTP, K_temp);  // K_temp = P_est_priori * C'
    mat3_mul(K_temp, S_inv, K_kal);       // K_kal = K_temp * S^-1

    // ACTUALIZACIÓN DEL ESTADO 
    // x_est = x_est_priori + K*innov
    float K_innov[3];
    mat3_mul_vec(K_kal, innov, K_innov);  // K*innov
    for (int i = 0; i < 3; i++)
      x_est[i] = x_est_priori[i] + K_innov[i];

    // ACTUALIZACIÓN DE LA COVARIANZA 
    // P_est = (I - K*C) * P_est_priori
    float IminusK[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        IminusK[i][j] = I3[i][j] - K_kal[i][j];  // I - K*C

    mat3_mul(IminusK, P_est_priori, P_est);  // P_est = (I-K*C)*P_est_priori
  }

  // ESTADOS ESTIMADOS FINALES 
  float theta_hat = x_est[0];   // ángulo estimado
  float omega_hat = x_est[1];   // velocidad angular estimada
  float xhat_riel = x_est[2];   // posición en la riel estimada

  // 3) LEY DE CONTROL
  float u_sinin = -k_theta * theta_hat
                  -k_omega * omega_hat
                  -k_x     * xhat_riel;

  if (!isfinite(u_sinin)) {
    analogWrite(PIN_ENAB, 0);
    digitalWrite(PIN_HORARIO, LOW);
    digitalWrite(PIN_ANTIHORARIO, LOW);

    float t_s = (micros() - t_start_us) / 1e6f;
    emitirCSV(t_s, theta_deg, omega_deg_s, x_riel_cm,
              0.0f, 0.0f, theta_hat, omega_hat, xhat_riel);
    u_model_prev = 0.0f;
    return;
  }

  // 4) INTERPOLACIÓN A VOLTAJE FIRMADO (-5..+5) y SALIDA MOTOR 
  // Convertir u_sinin (señal cruda) a voltaje firmado:
  float u_signed_volt = mapSignedVolt(u_sinin); // en [-5, +5], ahora sin truncar aún

  // Deadzone con histeresis 
  float abs_u = fabs(u_signed_volt);
  if (in_deadzone) {
    // estamos en deadzone; solo salimos si la magnitud supera U_RESUME_VOLT
    if (abs_u >= U_RESUME_VOLT) {
      in_deadzone = false;
    } else {
      // seguimos en deadzone -> forzamos a cero
      u_signed_volt = 0.0f;
    }
  } else {
    // no estábamos en deadzone; entramos si |u| cae por debajo de U_DEAD_VOLT
    if (abs_u < U_DEAD_VOLT) {
      in_deadzone = true;
      u_signed_volt = 0.0f;
    }
  }

  // Dirección según signo y PWM con magnitud
  if (u_signed_volt > 0.0f) {
    digitalWrite(PIN_ANTIHORARIO, HIGH);
    digitalWrite(PIN_HORARIO, LOW);
  } else if (u_signed_volt < 0.0f) {
    digitalWrite(PIN_ANTIHORARIO, LOW);
    digitalWrite(PIN_HORARIO, HIGH);
  } else {
    digitalWrite(PIN_ANTIHORARIO, LOW);
    digitalWrite(PIN_HORARIO, LOW);
  }

  float u_volt_mag = fabs(u_signed_volt); // magnitud 0..5 V para PWM
  analogWrite(PIN_ENAB, voltsA_PWM(u_volt_mag));

  // Actualizamos la entrada del modelo 
  u_model_prev = u_signed_volt;


  // subir datitos
  float t_s = (micros() - t_start_us) / 1e6f;
  emitirCSV(t_s, theta_deg, omega_deg_s, x_riel_cm,
            u_sinin, u_volt_mag, theta_hat, omega_hat, xhat_riel);
}
