#include <Arduino.h>
#include <Encoder.h>
#include <Fuzzy.h>

const uint8_t PIN_ENAB        = 9;
const uint8_t PIN_HORARIO     = 4;
const uint8_t PIN_ANTIHORARIO = 7;

const uint8_t PIN_ENC_A = 19;
const uint8_t PIN_ENC_B = 18;
const int PPR = 20;

Encoder encoder(PIN_ENC_A, PIN_ENC_B);

const float QUAD_X = 4.0f;
const float COUNTS_PER_REV = PPR * QUAD_X;
const float DEG_PER_COUNT = 360.0f / COUNTS_PER_REV;

static inline float countsToDeg(long c) { return c * DEG_PER_COUNT; }

// CONTROL DIFUSO

float SP_deg = 0.0f;  // objetivo = ángulo vertical

float prev_err = 0.0f;
float der_filt = 0.0f;
float alpha = 0.65f;

const int PWM_MIN = 60;
const int PWM_DEADBAND = 10;
const float K_FUZZY = 1.3f;

unsigned long t_prev_us = 0;

Fuzzy *fuzzy = new Fuzzy();


void applyMotor(int u) {
  u = constrain(u, -255, 255);

  if (abs(u) < PWM_DEADBAND) {
    analogWrite(PIN_ENAB, 0);
    digitalWrite(PIN_HORARIO, LOW);
    digitalWrite(PIN_ANTIHORARIO, LOW);
    return;
  }

  int pwmAbs = abs(u);
  if (pwmAbs < PWM_MIN) pwmAbs = PWM_MIN;

  if (u > 0) {
    digitalWrite(PIN_HORARIO, HIGH);
    digitalWrite(PIN_ANTIHORARIO, LOW);
    analogWrite(PIN_ENAB, pwmAbs);
  } else {
    digitalWrite(PIN_HORARIO, LOW);
    digitalWrite(PIN_ANTIHORARIO, HIGH);
    analogWrite(PIN_ENAB, pwmAbs);
  }
}


void setupFuzzy() {

  // -------- INPUT 1--------
  FuzzyInput *err = new FuzzyInput(1);

  FuzzySet *ERR_NL = new FuzzySet(-40, -40, -20, -5);
  FuzzySet *ERR_ZE = new FuzzySet(-5, 0, 0, 5);
  FuzzySet *ERR_PL = new FuzzySet(5, 20, 40, 40);

  err->addFuzzySet(ERR_NL);
  err->addFuzzySet(ERR_ZE);
  err->addFuzzySet(ERR_PL);
  fuzzy->addFuzzyInput(err);

  // -------- INPUT 2----------
  FuzzyInput *derr = new FuzzyInput(2);

  FuzzySet *DER_N = new FuzzySet(-60, -60, -20, -5);
  FuzzySet *DER_ZE = new FuzzySet(-5, 0, 0, 5);
  FuzzySet *DER_P = new FuzzySet(5, 20, 60, 60);

  derr->addFuzzySet(DER_N);
  derr->addFuzzySet(DER_ZE);
  derr->addFuzzySet(DER_P);
  fuzzy->addFuzzyInput(derr);

  // ---------- OUTPUT---------
  FuzzyOutput *outPWM = new FuzzyOutput(1);

  FuzzySet *PWM_NL = new FuzzySet(-255, -255, -200, -150);
  FuzzySet *PWM_NM = new FuzzySet(-180, -140, -90, -40);
  FuzzySet *PWM_ZE = new FuzzySet(-20, 0, 0, 20);
  FuzzySet *PWM_PM = new FuzzySet(40, 90, 140, 180);
  FuzzySet *PWM_PL = new FuzzySet(150, 200, 255, 255);

  outPWM->addFuzzySet(PWM_NL); 
  outPWM->addFuzzySet(PWM_NM);
  outPWM->addFuzzySet(PWM_ZE);
  outPWM->addFuzzySet(PWM_PM);
  outPWM->addFuzzySet(PWM_PL);
  fuzzy->addFuzzyOutput(outPWM);

  // REGLAS
  {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinSingle(ERR_NL);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(PWM_PL);
    fuzzy->addFuzzyRule(new FuzzyRule(1, a, c));
  }
  {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinSingle(ERR_PL);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(PWM_NL);
    fuzzy->addFuzzyRule(new FuzzyRule(2, a, c));
  }
  {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(ERR_ZE, DER_P);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(PWM_NL);
    fuzzy->addFuzzyRule(new FuzzyRule(3, a, c));
  }
  {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(ERR_ZE, DER_N);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(PWM_PL);
    fuzzy->addFuzzyRule(new FuzzyRule(4, a, c));
  }
  {
    FuzzyRuleAntecedent *a = new FuzzyRuleAntecedent();
    a->joinWithAND(ERR_ZE, DER_ZE);
    FuzzyRuleConsequent *c = new FuzzyRuleConsequent();
    c->addOutput(PWM_ZE);
    fuzzy->addFuzzyRule(new FuzzyRule(5, a, c));
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_ENAB, OUTPUT);
  pinMode(PIN_HORARIO, OUTPUT);
  pinMode(PIN_ANTIHORARIO, OUTPUT);

  encoder.write(0);
  t_prev_us = micros();

  setupFuzzy();
  Serial.println("t,angle,err,derr,pwm");
}

void loop() {

  unsigned long now_us = micros();
  float dt = (now_us - t_prev_us) / 1e6f;
  if (dt <= 0 || dt > 0.05f) dt = 0.01f;
  t_prev_us = now_us;

  long counts = encoder.read();
  float angle_deg = countsToDeg(counts);

  float err = SP_deg - angle_deg;

  float derr = (err - prev_err) / dt;
  prev_err = err;

  der_filt = alpha * der_filt + (1.0f - alpha) * derr;

  fuzzy->setInput(1, err);
  fuzzy->setInput(2, der_filt);
  fuzzy->fuzzify();

  float u = fuzzy->defuzzify(1);
  u *= K_FUZZY;

  applyMotor((int)u);

  Serial.print(millis()/1000.0f); Serial.print(",");
  Serial.print(angle_deg); Serial.print(",");
  Serial.print(err); Serial.print(",");
  Serial.print(der_filt); Serial.print(",");
  Serial.println((int)u);

  delay(5);
}
