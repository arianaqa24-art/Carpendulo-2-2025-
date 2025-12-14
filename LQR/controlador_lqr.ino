#include <Arduino.h>
#include <math.h>
#include <Encoder.h>
#include <Ultrasonic.h>
#ifdef SP
#undef SP
#endif

//Parámetros de controlador
//Las ganancia de los estadoss: u = k_theta*(SP_deg - theta_deg) + k_omega*(-omega_deg_s) + k_i*I

float k_theta = 0.50f;   // ganancia sobre el error de posición (grados)
float k_omega = 0.030f;  // ganancia sobre la velocidad (deg/s)
float k_x     = 0.05f;   // ganancia distancia 
float k_i     = 0.08f;   // ganancia integral para eliminar error en régimen permanente

const uint8_t PIN_TRIG        = 5;
const uint8_t PIN_ECHO        = 6;
const uint8_t PIN_ENAB        = 9;
const uint8_t PIN_HORARIO     = 4;
const uint8_t PIN_ANTIHORARIO = 7;
const uint8_t PIN_ENC_A       = 19;
const uint8_t PIN_ENC_B       = 18;
const int     PPR             = 20;

Encoder    encoder(PIN_ENC_A, PIN_ENC_B);
Ultrasonic ultra(PIN_TRIG, PIN_ECHO);

const float QUAD_X = 4.0f;
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
  if (cm <= 0) return INFINITY;
  return cm;
}

//Ref de la posicion de la riell 
const float X_REF_CM = 15.0f;

unsigned long t_start_us = 0;


static unsigned long t_prev_us   = 0;
static long          counts_prev = 0;

void emitirCSV(float t_s, float theta_deg, float omega_deg_s,
               float x_riel_cm, float u_sinin, float u_volt) {
  Serial.print(t_s, 6);        Serial.print(',');
  Serial.print(theta_deg, 6);  Serial.print(',');
  Serial.print(omega_deg_s, 6);Serial.print(',');
  Serial.print(x_riel_cm, 6);  Serial.print(',');
  Serial.print(u_sinin, 6);    Serial.print(',');
  Serial.println(u_volt, 6);
}

//Moditos de homing , hold y control
enum Mode { MODE_CONTROL = 0, MODE_HOMING = 1, MODE_HOLD = 2 };
Mode mode = MODE_CONTROL;

//homing
const float homing_pwm_volts    = 2.0f;    // voltaje aplicado durante homing (magnitud)
const float homing_tol_cm       = 0.5f;    // tolerancia para considerar 'en posición' (cm)
const unsigned long hold_duration_ms = 500; // tiempo para quedarse quieto antes de reintentar control (ms)

//Parametros para buscar cuando el ultrasonico no sirve
const unsigned long search_max_ms = 2000;   // máximo tiempo de búsqueda para recuperar lectura (ms)
unsigned long search_start_ms = 0;          // cuando empezó la búsqueda
bool  search_active = false;

unsigned long hold_start_ms = 0;

void setup() {
  pinMode(PIN_HORARIO, OUTPUT);
  pinMode(PIN_ANTIHORARIO, OUTPUT);
  pinMode(PIN_ENAB, OUTPUT);

  digitalWrite(PIN_ANTIHORARIO, LOW);
  digitalWrite(PIN_HORARIO, LOW);

  analogWrite(PIN_ENAB, voltsA_PWM(0));
  float distancia = leerDistanciaM();

  while (isfinite(distancia) &&
         (distancia < (X_REF_CM - 0.5f) || distancia > (X_REF_CM + 0.5f))) {

    if (distancia < X_REF_CM) {
      //Se aleja cuando se pone muy cerca del sensor
      digitalWrite(PIN_ANTIHORARIO, HIGH);
      digitalWrite(PIN_HORARIO, LOW);
    } else {
      //Se acerca
      digitalWrite(PIN_ANTIHORARIO, LOW);
      digitalWrite(PIN_HORARIO, HIGH);
    }
    analogWrite(PIN_ENAB, voltsA_PWM(homing_pwm_volts));

    distancia = leerDistanciaM();
    delay(20); //espera para no saturar al sensor
  }

  analogWrite(PIN_ENAB, 0);

  encoder.write(0L);
  counts_prev = 0;

  Serial.begin(9600);
  while (!Serial) { /*  Por Seguridad */ }
  Serial.println("t_s,theta_deg,omega_deg_s,x_riel_cm,u_sinin,u_volt");

  t_start_us = micros();
  t_prev_us  = t_start_us;
}

void loop() {
  // --- Cálculo de dt real entre iteraciones ---
  unsigned long t_now_us = micros();
  float dt_s = (t_now_us - t_prev_us) / 1e6f;
  if (dt_s <= 0.0f || dt_s > 0.05f) {
    // Si pasó demasiado tiempo (bloqueo), asumir 10 ms
    dt_s = 0.01f;
  }
  t_prev_us = t_now_us;

  // ==================== 1) Medición de estados ====================
  // 1a) Encoder → theta y omega
  long counts = encoder.read();
  long delta_counts = counts - counts_prev;
  counts_prev = counts;

  float theta_deg    = countsToDeg((float)counts);               // x1
  float omega_deg_s  = (delta_counts * DEG_PER_COUNT) / dt_s;    // x2

  // 1b) Ultrasónico → posición en la riel (respecto al centro)
  float distancia_cm = leerDistanciaM();
  float x_riel_cm    = 0.0f; // x3
  bool  distancia_ok = isfinite(distancia_cm);

  if (distancia_ok) {
    x_riel_cm = distancia_cm - X_REF_CM; // 0 cuando está en X_REF_CM
  }

  // ---------- State machine para homing / hold / control ----------
  bool fuera_zona_segura = !distancia_ok || distancia_cm < 3.5f || distancia_cm > 28.5f;

  if (mode == MODE_CONTROL) {
    if (fuera_zona_segura) {
      // Entrar en homing para llevar el carro a X_REF_CM
      mode = MODE_HOMING;
      // reset búsqueda
      search_active = false;
      search_start_ms = 0;
    }
  }

  if (mode == MODE_HOMING) {
    // Si no hay lectura válida, intentamos una búsqueda breve moviendo ligeramente
    if (!distancia_ok) {
      unsigned long now_ms = millis();
      if (!search_active) {
        search_active = true;
        search_start_ms = now_ms;
      }
      unsigned long elapsed = now_ms - search_start_ms;

      if (elapsed <= search_max_ms) {
        // realizar pequeños "bursts" alejándose para intentar recuperar lectura
        // aplicamos homing_pwm_volts en sentido ANTIHORARIO (alejarse)
        digitalWrite(PIN_ANTIHORARIO, HIGH);
        digitalWrite(PIN_HORARIO, LOW);
        analogWrite(PIN_ENAB, voltsA_PWM(homing_pwm_volts));
        // dejamos que el loop siga y vuelva a leer la distancia
        float t_s = (micros() - t_start_us) / 1e6f;
        emitirCSV(t_s, theta_deg, omega_deg_s, x_riel_cm, 0.0f, homing_pwm_volts);
        return; // esperamos a la siguiente iteración para leer de nuevo
      } else {
        // no pudimos recuperar lectura en el tiempo máximo -> ir a HOLD seguro
        search_active = false;
        analogWrite(PIN_ENAB, 0);
        digitalWrite(PIN_HORARIO, LOW);
        digitalWrite(PIN_ANTIHORARIO, LOW);
        mode = MODE_HOLD;
        hold_start_ms = millis();
      }
    } else {
      // Tenemos lectura válida: mover hacia X_REF_CM hasta estar dentro de tolerancia
      float error_cm = distancia_cm - X_REF_CM; // positivo -> estamos más lejos (necesitamos acercar)
      if (fabs(error_cm) <= homing_tol_cm) {
        // Llegó al punto de referencia
        analogWrite(PIN_ENAB, 0);
        digitalWrite(PIN_HORARIO, LOW);
        digitalWrite(PIN_ANTIHORARIO, LOW);
        mode = MODE_HOLD;
        hold_start_ms = millis();
      } else {
        // Decidir sentido: si distancia > X_REF -> acercar (HORARIO en tu esquema de setup),
        // si distancia < X_REF -> alejar (ANTIHORARIO)
        if (distancia_cm > X_REF_CM) {
          // acercar
          digitalWrite(PIN_ANTIHORARIO, LOW);
          digitalWrite(PIN_HORARIO, HIGH);
        } else {
          // alejar
          digitalWrite(PIN_ANTIHORARIO, HIGH);
          digitalWrite(PIN_HORARIO, LOW);
        }
        analogWrite(PIN_ENAB, voltsA_PWM(homing_pwm_volts));

        // Emitir log de homing para depuración (u_sinin = 0 porque no estamos controlando)
        float t_s = (micros() - t_start_us) / 1e6f;
        emitirCSV(t_s, theta_deg, omega_deg_s, x_riel_cm, 0.0f, homing_pwm_volts);
        return; // dejamos que el loop vuelva a leer en la siguiente iteración
      }
    }
  }

  if (mode == MODE_HOLD) {
    // Freno motor y esperar un tiempo de asentamiento corto
    analogWrite(PIN_ENAB, 0);
    digitalWrite(PIN_HORARIO, LOW);
    digitalWrite(PIN_ANTIHORARIO, LOW);

    // Emitir log mientras estamos en HOLD (u = 0)
    float t_s = (micros() - t_start_us) / 1e6f;
    emitirCSV(t_s, theta_deg, omega_deg_s, x_riel_cm, 0.0f, 0.0f);

    // Si pasó el tiempo de hold, comprobar si podemos volver a Control
    if ((millis() - hold_start_ms) >= hold_duration_ms) {
      // Solo volvemos a control si la lectura es válida y está dentro de la zona segura
      if (distancia_ok && distancia_cm >= 3.5f && distancia_cm <= 28.5f) {
        mode = MODE_CONTROL;
        // dejamos que el loop siga y el regulador opere en la misma iteración
        // (no hacemos return para que el código de control continúe más abajo)
      } else {
        // si aun no está bien, volver a HOMING para intentar posicionar otra vez
        mode = MODE_HOMING;
      }
    } else {
      // todavía en periodo de hold -> no ejecutar control
      return;
    }
  }

  // Si llegamos aquí, estamos en MODE_CONTROL (o acabamos de saltar desde HOLD a CONTROL).
  // ==================== 3) Ley de control LQR-like (3 estados) ====================
  // estados:
  // x1 = theta_deg
  // x2 = omega_deg_s
  // x3 = x_riel_cm
  //
  // Queremos llevar: theta_deg → 0, omega_deg_s → 0, x_riel_cm → 0
  //
  // u = -k_theta * theta_deg - k_omega * omega_deg_s - k_x * x_riel_cm

  float u_sinin = -k_theta * theta_deg
                  -k_omega * omega_deg_s
                  -k_x     * x_riel_cm;

  // Protección contra NaN o Infs
  if (!isfinite(u_sinin)) {
    analogWrite(PIN_ENAB, 0);
    digitalWrite(PIN_HORARIO, LOW);
    digitalWrite(PIN_ANTIHORARIO, LOW);

    float t_s = (micros() - t_start_us) / 1e6f;
    emitirCSV(t_s, theta_deg, omega_deg_s, x_riel_cm, 0.0f, 0.0f);
    return;
  }

  // ==================== 4) Dirección del motor y saturación ====================
  float u_volt = 0.0f;

  if (u_sinin > 0.0f) {
    // Sentido antihorario
    digitalWrite(PIN_ANTIHORARIO, HIGH);
    digitalWrite(PIN_HORARIO, LOW);
    u_volt = u_sinin;
  } else if (u_sinin < 0.0f) {
    // Sentido horario
    digitalWrite(PIN_ANTIHORARIO, LOW);
    digitalWrite(PIN_HORARIO, HIGH);
    u_volt = -u_sinin; // magnitud
  } else {
    // Cero → apaga dirección
    digitalWrite(PIN_ANTIHORARIO, LOW);
    digitalWrite(PIN_HORARIO, LOW);
    u_volt = 0.0f;
  }

  // Saturación en 0–5 V
  if (u_volt > 5.0f) u_volt = 5.0f;
  if (u_volt < 0.0f) u_volt = 0.0f;

  // Enviar PWM
  analogWrite(PIN_ENAB, voltsA_PWM(u_volt));

  // ==================== 5) Log de datos ====================
  float t_s = (micros() - t_start_us) / 1e6f;
  emitirCSV(t_s, theta_deg, omega_deg_s, x_riel_cm, u_sinin, u_volt);
}
