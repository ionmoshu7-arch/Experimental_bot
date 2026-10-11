#include "Inventaire.h"

#include <Arduino.h>
#include "Bus.h"
#include "Config.h"
#include "Couleur.h"
#include "Distance.h"
#include "Moteurs.h"

static const uint8_t WHO_AM_I_REG = 0x75;
static const uint8_t WHO_AM_I_MPU6500 = 0x70;

// Plan d'adressage I2C (Suivi n.19). installe = "Fait" dans le suivi.
struct PeripheriqueI2C {
  uint8_t adresse;
  const char* nom;
  bool installe;
};

static const PeripheriqueI2C PLAN_I2C[] = {
  { 0x20, "MCP23017",          false },
  { 0x38, "DHT20",             false },
  { 0x3C, "OLED",              false },
  { 0x40, "PCA9685 #1 servos", false },
  { 0x41, "PCA9685 #2 LED",    false },
  { 0x48, "ADS1115",           false },
  { 0x50, "ESP32-S3-CAM",      false },
  { ADRESSE_IMU, "MPU-6500",   true  },
  { ADRESSE_MUX, "TCA9548A",   true  },
};
static const int NB_PLAN_I2C = sizeof(PLAN_I2C) / sizeof(PLAN_I2C[0]);

// Contenu prevu de chaque canal du multiplexeur (Connexions n.49-55)
static const char* PLAN_CANAUX[8] = {
  "VL53L1X avant", "VL53L0X arriere", "VL53L0X droite", "VL53L0X gauche",
  "TCS34725 sol gauche", "TCS34725 sol centre", "TCS34725 sol droit", "libre"
};

static const PeripheriqueI2C* chercherPlan(uint8_t adresse) {
  for (int i = 0; i < NB_PLAN_I2C; i++)
    if (PLAN_I2C[i].adresse == adresse) return &PLAN_I2C[i];
  return nullptr;
}

static bool scannerBusPrincipal() {
  Serial.println("--- Bus I2C principal (SDA=GPIO38, SCL=GPIO39) ---");

  // Canaux du mux fermes pour ne voir que le bus principal
  bool mux = muxPresent();
  if (mux) muxFermer();

  bool vu[128] = { false };
  for (uint8_t a = 1; a < 127; a++) {
    if (!busRepond(a)) continue;
    vu[a] = true;
    const PeripheriqueI2C* p = chercherPlan(a);
    Serial.printf("  0x%02X  %-20s %s\n", a, p ? p->nom : "INCONNU",
                  !p ? "<- hors plan d'adressage"
                     : p->installe ? "OK" : "present (non coche dans le suivi)");
  }
  for (int i = 0; i < NB_PLAN_I2C; i++) {
    if (PLAN_I2C[i].installe && !vu[PLAN_I2C[i].adresse])
      Serial.printf("  0x%02X  %-20s MANQUANT\n",
                    PLAN_I2C[i].adresse, PLAN_I2C[i].nom);
  }
  return mux;
}

static void scannerCanauxMux(bool mux) {
  Serial.println("--- Canaux du TCA9548A (0x73) ---");
  if (!mux) {
    Serial.println("  multiplexeur absent : aucun capteur de distance joignable");
    return;
  }

  // Ce qui repond deja canaux fermes appartient au bus principal
  muxFermer();
  bool principal[128] = { false };
  for (uint8_t a = 1; a < 127; a++) principal[a] = busRepond(a);

  for (uint8_t canal = 0; canal < 8; canal++) {
    muxChoisir(canal);
    Serial.printf("  canal %d  %-28s :", canal, PLAN_CANAUX[canal]);
    int n = 0;
    for (uint8_t a = 1; a < 127; a++) {
      if (principal[a] || a == ADRESSE_MUX) continue;
      if (busRepond(a)) {
        Serial.printf(" 0x%02X", a);
        n++;
      }
    }
    Serial.println(n ? "" : " vide");
  }
  muxFermer();
}

static void verifierIMU() {
  uint8_t who = 0;
  Serial.println("--- Gyroscope ---");
  if (!busLireRegistre(ADRESSE_IMU, WHO_AM_I_REG, who)) {
    Serial.println("  MPU-6500 (0x68)      ne repond pas");
    return;
  }
  Serial.printf("  WHO_AM_I = 0x%02X  %s\n", who,
                who == WHO_AM_I_MPU6500 ? "(MPU-6500 confirme)"
                                        : "<- attendu 0x70 pour un MPU-6500");
}

static void afficherCablageFixe() {
  Serial.println("--- Cablage (non detectable, d'apres le plan) ---");
  for (auto &m : moteurs) {
    Serial.printf("  %-18s PWM=GPIO%-2d IN1=GPIO%-2d IN2=GPIO%-2d  enc A=GPIO%d B=GPIO%d\n",
                  m.nom, m.pwm, m.in1, m.in2, m.encA, m.encB);
  }
  Serial.println("  STBY=GPIO16 (2 x TB6612FNG)  - droit #1 : M1/M3, gauche #2 : M2/M4");
  Serial.println("  Inversions logicielles : IN1/IN2 de M1 et M4, phases A/B de M2 et M4");
  Serial.printf("  Encodeurs : %.0f impulsions/tour, %.3f mm/impulsion\n",
                IMPULSIONS_PAR_TOUR, MM_PAR_IMPULSION);

  Serial.print("  Niveaux encodeurs au repos (A,B) :");
  for (auto &m : moteurs)
    Serial.printf(" %d,%d", digitalRead(m.encA), digitalRead(m.encB));
  Serial.println();
}

void inventaireAfficher() {
  Serial.println("=========== INVENTAIRE MATERIEL ===========");
  bool mux = scannerBusPrincipal();
  scannerCanauxMux(mux);
  verifierIMU();
  afficherCablageFixe();
}

void inventaireAfficherDistance() {
  Serial.println("--- Capteurs de distance ---");
  for (int i = 0; i < 4; i++) {
    const CapteurToF &c = tof[i];
    Serial.printf("  canal %d  %-8s %s  ", i, c.nom, c.modele);
    if (!c.present)     Serial.println("ABSENT");
    else if (!c.fiable) Serial.println("present, NON FIABLE (ignore pour decider)");
    else                Serial.printf("OK, portee %d mm\n", c.porteeMm);
  }
}

void inventaireAfficherCouleur() {
  Serial.println("--- Capteurs de couleur ---");
  for (auto &k : couleurs)
    Serial.printf("  canal %d  %-10s TCS34725  %s\n", k.canal, k.nom,
                  k.present ? "OK" : "ABSENT");
}
