#include "Moteurs.h"

#include "Config.h"

Moteur moteurs[4] = {
  { M1_PWM, M1_IN1, M1_IN2, 0, "M1 avant-droit",    ENC1_A, ENC1_B },
  { M2_PWM, M2_IN1, M2_IN2, 1, "M2 avant-gauche",   ENC2_A, ENC2_B },
  { M3_PWM, M3_IN1, M3_IN2, 2, "M3 arriere-droit",  ENC3_A, ENC3_B },
  { M4_PWM, M4_IN1, M4_IN2, 3, "M4 arriere-gauche", ENC4_A, ENC4_B },
};

static const int8_t TABLE_QUAD[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

static unsigned long dernierCalculVitesse = 0;

static void IRAM_ATTR isrEncodeur(void* arg) {
  Moteur* m = (Moteur*) arg;
  uint8_t s = (digitalRead(m->encA) << 1) | digitalRead(m->encB);
  m->compte += TABLE_QUAD[(m->etatPrec << 2) | s];
  m->etatPrec = s;
}

static void pwmEcrire(const Moteur &m, int v) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(m.pwm, v);
#else
  ledcWrite(m.canalPwm, v);
#endif
}

void moteursInit() {
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);
  for (int i = 0; i < 4; i++) {
    Moteur &m = moteurs[i];
    pinMode(m.in1, OUTPUT);
    pinMode(m.in2, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(m.pwm, PWM_FREQ, PWM_RES);
#else
    ledcSetup(m.canalPwm, PWM_FREQ, PWM_RES);
    ledcAttachPin(m.pwm, m.canalPwm);
#endif
    moteurAppliquer(i, 0);

    pinMode(m.encA, INPUT);
    pinMode(m.encB, INPUT);
    m.etatPrec = (digitalRead(m.encA) << 1) | digitalRead(m.encB);
    attachInterruptArg(digitalPinToInterrupt(m.encA), isrEncodeur, &m, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(m.encB), isrEncodeur, &m, CHANGE);
  }
  dernierCalculVitesse = millis();
}

void moteurAppliquer(uint8_t i, int v) {
  Moteur &m = moteurs[i];
  v = constrain(v, -255, 255);
  m.consigne = v;
  digitalWrite(m.in1, v > 0 ? HIGH : LOW);
  digitalWrite(m.in2, v < 0 ? HIGH : LOW);
  pwmEcrire(m, abs(v));
}

void moteursStop() {
  for (int i = 0; i < 4; i++) moteurAppliquer(i, 0);
}

void moteursActiver(bool actif) {
  digitalWrite(STBY_PIN, actif ? HIGH : LOW);
}

bool moteursActive() {
  return digitalRead(STBY_PIN) == HIGH;
}

bool moteursActifs() {
  for (auto &m : moteurs) if (m.consigne != 0) return true;
  return false;
}

void moteursCotes(int droite, int gauche) {
  moteurAppliquer(M1, droite);
  moteurAppliquer(M3, droite);
  moteurAppliquer(M2, gauche);
  moteurAppliquer(M4, gauche);
}

void avancer(int v)       { moteursCotes(v, v); }
void reculer(int v)       { moteursCotes(-v, -v); }
void tournerDroite(int v) { moteursCotes(-v, v); }
void tournerGauche(int v) { moteursCotes(v, -v); }

long moteurCompte(uint8_t i) {
  noInterrupts();
  long c = moteurs[i].compte;
  interrupts();
  return c;
}

long compteMoyen() {
  long total = 0;
  for (int i = 0; i < 4; i++) total += abs(moteurCompte(i));
  return total / 4;
}

void moteursRemettreAZero() {
  noInterrupts();
  for (auto &m : moteurs) m.compte = 0;
  interrupts();
  for (auto &m : moteurs) m.comptePrec = 0;
}

void moteursCalculerVitesses() {
  unsigned long maintenant = millis();
  unsigned long ecart = maintenant - dernierCalculVitesse;
  if (ecart < 100) return;
  dernierCalculVitesse = maintenant;
  for (int i = 0; i < 4; i++) {
    Moteur &m = moteurs[i];
    long c = moteurCompte(i);
    float v = (c - m.comptePrec) * MM_PAR_IMPULSION * 1000.0 / ecart;
    m.comptePrec = c;
    m.vitesse = 0.5 * m.vitesse + 0.5 * v;
  }
}
