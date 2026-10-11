/*
 * 15_TEST_CAPTEURS.cpp
 * ------------------------------------------------------------
 * LECTURE DE TOUS LES CAPTEURS - AFFICHAGE SUR LE MONITEUR SERIE
 *
 *   - 4 capteurs de distance (TCA9548A canaux 0 a 3)
 *       0 : VL53L1X avant      1 : VL53L0X arriere
 *       2 : VL53L0X droite     3 : VL53L0X gauche
 *   - 3 capteurs de couleur TCS34725 (canaux 4, 5, 6)
 *       4 : sol gauche   5 : sol centre   6 : sol droit
 *   - Gyroscope / accelerometre MPU-6500 (0x68, bus principal)
 *
 * LES MOTEURS RESTENT ARRETES (STBY a LOW) : le rover ne bouge pas.
 *
 * SORTIES SUR LE PORT SERIE (115200 bauds) :
 *   - un bloc lisible chaque seconde (moniteur serie de VS Code)
 *   - une ligne "[DATA]{...json...}" 5 fois par seconde, lue par
 *     la page web/rover_moniteur.html
 *
 * COMMANDES (taper la lettre dans le moniteur serie ou la page) :
 *   c : recalibrer le gyroscope (ne pas bouger le rover)
 *   z : remettre le cap a zero
 *
 * Le TCS34725 est pilote en registres directs : pas de bibliotheque.
 * ------------------------------------------------------------
 */

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <VL53L0X.h>
#include "FastIMU.h"

// ================== BROCHAGE ==================

#define SDA_PIN 38
#define SCL_PIN 39
#define I2C_CLOCK_HZ 100000

#define ADRESSE_MUX 0x73
#define ADRESSE_IMU 0x68
#define ADRESSE_TCS 0x29

#define STBY_PIN 16
const uint8_t BROCHES_MOTEURS[] = { 2, 4, 1, 5, 6, 7, 10, 11, 12, 13, 15, 14 };

// ================== REGLAGES ==================

const unsigned long PERIODE_TEXTE_MS = 1000;   // 0 pour desactiver
const unsigned long PERIODE_JSON_MS = 200;     // 0 pour desactiver

// VL53L0X : 8190/8191 = pas de mesure (code d'erreur, pas une distance)
const int VL53L0X_CODE_ERREUR = 8190;

// TCS34725 : temps d'integration = (256 - ATIME) x 2,4 ms
const uint8_t TCS_ATIME = 0xEB;                // 21 cycles = 50,4 ms
const uint8_t TCS_GAIN = 0x01;                 // 0=1x 1=4x 2=16x 3=60x
const uint8_t TCS_GAINS[4] = { 1, 4, 16, 60 };

// Classement approximatif de la couleur du sol : a ajuster selon
// l'eclairage et la hauteur des capteurs (voir lum dans le bloc).
const float SEUIL_NOIR = 0.03;    // luminosite relative en dessous = noir
const float SEUIL_BLANC = 0.30;   // au dessus et sans dominante = blanc
const float SEUIL_DOMINANTE = 0.42;

// ================== CAPTEURS DE DISTANCE ==================

enum EtatMesure { M_ABSENT, M_JAMAIS_LU, M_OK, M_HORS_PORTEE, M_TIMEOUT };

struct CapteurToF {
  const char* nom;
  const char* modele;
  int porteeMm;      // portee reelle mesuree (feuille Calibration)
  bool fiable;       // capteur droit DEFAILLANT au 01.09.26
  bool present;
  int brut;
  uint8_t statutL1X;
  EtatMesure etat;
};

VL53L1X tofAvant;
VL53L0X tofArriere, tofDroite, tofGauche;

CapteurToF tof[4] = {
  { "avant",   "VL53L1X", 4000, true,  false, -1, 0, M_JAMAIS_LU },
  { "arriere", "VL53L0X", 1200, true,  false, -1, 0, M_JAMAIS_LU },
  { "droite",  "VL53L0X", 1200, false, false, -1, 0, M_JAMAIS_LU },
  { "gauche",  "VL53L0X",  600, true,  false, -1, 0, M_JAMAIS_LU }
};

// ================== CAPTEURS DE COULEUR ==================

struct CapteurCouleur {
  const char* nom;
  uint8_t canal;
  bool present;
  bool valide;       // au moins une mesure lue
  uint16_t c, r, g, b;
  float lux;
  int cct;           // temperature de couleur en kelvin (0 = inconnue)
  float lum;         // c / comptage maximal, 0 a 1
  bool sature;
  const char* couleur;
};

CapteurCouleur couleurs[3] = {
  { "sol_gauche", 4 }, { "sol_centre", 5 }, { "sol_droit", 6 }
};

// ================== GYROSCOPE ==================

MPU6500 IMU;
calData calib = { 0 };
bool imuPresent = false;
AccelData accel;
GyroData gyro;
float temperatureImu = 0;
float biaisGyroZ = 0;
float cap = 0;
unsigned long dernierEchantillonGyro = 0;

// ================== ETAT ==================

bool muxPresent = false;
unsigned long dernierTexte = 0, dernierJson = 0;
unsigned long nbBoucles = 0, dernierCalculHz = 0;
unsigned long boucleHz = 0;

// ================== I2C ET MULTIPLEXEUR ==================

void choisirCanal(uint8_t canal) {
  Wire.beginTransmission(ADRESSE_MUX);
  Wire.write(1 << canal);
  Wire.endTransmission();
}

bool repond(uint8_t adresse) {
  Wire.beginTransmission(adresse);
  return Wire.endTransmission() == 0;
}

// ================== GYROSCOPE ==================

void mettreAJourIMU() {
  unsigned long maintenant = micros();
  float dt = (maintenant - dernierEchantillonGyro) / 1000000.0;
  dernierEchantillonGyro = maintenant;
  if (!imuPresent) return;

  IMU.update();
  IMU.getGyro(&gyro);
  IMU.getAccel(&accel);
  temperatureImu = IMU.getTemp();
  // Convention : rotation a DROITE = cap NEGATIF
  cap += (gyro.gyroZ - biaisGyroZ) * dt;
}

void calibrerGyro() {
  if (!imuPresent) return;
  Serial.println("Calibration du gyroscope - NE PAS BOUGER LE ROVER...");
  double somme = 0;
  for (int i = 0; i < 1500; i++) {
    IMU.update();
    IMU.getGyro(&gyro);
    somme += gyro.gyroZ;
    delay(1);
  }
  biaisGyroZ = somme / 1500.0;
  cap = 0;
  dernierEchantillonGyro = micros();
  Serial.printf("Biais axe Z : %.4f deg/s\n", biaisGyroZ);
}

// ================== DISTANCE ==================

void lireDistance(uint8_t canal) {
  CapteurToF &t = tof[canal];
  if (!t.present) { t.etat = M_ABSENT; return; }

  choisirCanal(canal);
  int d;
  bool timeout;
  if (canal == 0) {
    d = tofAvant.read();
    timeout = tofAvant.timeoutOccurred();
    t.statutL1X = tofAvant.ranging_data.range_status;
  } else {
    VL53L0X &s = (canal == 1) ? tofArriere : (canal == 2) ? tofDroite : tofGauche;
    d = s.readRangeContinuousMillimeters();
    timeout = s.timeoutOccurred();
  }
  t.brut = d;
  if (timeout) t.etat = M_TIMEOUT;
  else if (canal != 0 && d >= VL53L0X_CODE_ERREUR) t.etat = M_HORS_PORTEE;
  else t.etat = M_OK;
}

void initVL53L0X(VL53L0X &s, uint8_t canal) {
  choisirCanal(canal);
  s.setBus(&Wire);
  s.setTimeout(200);
  if (s.init()) {
    s.startContinuous();
    tof[canal].present = true;
  }
}

// Code court pour le JSON et le texte
const char* codeEtat(const CapteurToF &t) {
  if (t.etat == M_ABSENT) return "abs";
  if (!t.fiable) return "nf";
  switch (t.etat) {
    case M_OK:          return "ok";
    case M_HORS_PORTEE: return "hp";
    case M_TIMEOUT:     return "err";
    default:            return "--";
  }
}

void texteDistance(const CapteurToF &t, char* buf, size_t n) {
  switch (t.etat) {
    case M_ABSENT:      snprintf(buf, n, "ABSENT"); return;
    case M_JAMAIS_LU:   snprintf(buf, n, "--"); return;
    case M_TIMEOUT:     snprintf(buf, n, "%sERR", t.fiable ? "" : "NF:"); return;
    case M_HORS_PORTEE:
      if (t.fiable) snprintf(buf, n, ">%d mm", t.porteeMm);
      else          snprintf(buf, n, "NF:%d", t.brut);
      return;
    case M_OK:
      snprintf(buf, n, "%s%d mm", t.fiable ? "" : "NF:", t.brut);
      return;
  }
}

// ================== COULEUR (TCS34725) ==================

bool tcsEcrire(uint8_t reg, uint8_t valeur) {
  Wire.beginTransmission(ADRESSE_TCS);
  Wire.write(0x80 | reg);
  Wire.write(valeur);
  return Wire.endTransmission() == 0;
}

bool tcsLire(uint8_t reg, uint8_t* dest, uint8_t n) {
  Wire.beginTransmission(ADRESSE_TCS);
  Wire.write(0xA0 | reg);          // commande + auto-increment
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((uint8_t) ADRESSE_TCS, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) dest[i] = Wire.read();
  return true;
}

uint16_t comptageMaxTcs() {
  uint32_t m = (256 - TCS_ATIME) * 1024UL;
  return m > 65535 ? 65535 : m;
}

void initCouleur(CapteurCouleur &k) {
  choisirCanal(k.canal);
  uint8_t id = 0;
  // 0x44 = TCS34721/34725, 0x4D = TCS34723/34727
  if (!tcsLire(0x12, &id, 1) || (id != 0x44 && id != 0x4D)) return;
  tcsEcrire(0x01, TCS_ATIME);
  tcsEcrire(0x0F, TCS_GAIN);
  tcsEcrire(0x00, 0x01);           // PON
  delay(3);
  tcsEcrire(0x00, 0x03);           // PON + AEN
  k.present = true;
  k.couleur = "--";
}

const char* classerCouleur(const CapteurCouleur &k) {
  if (k.sature) return "sature";
  if (k.lum < SEUIL_NOIR) return "noir";
  float somme = (float) k.r + k.g + k.b;
  if (somme <= 0) return "noir";
  float fr = k.r / somme, fg = k.g / somme, fb = k.b / somme;
  if (fr > SEUIL_DOMINANTE && fg > 0.30) return "jaune";
  if (fr > SEUIL_DOMINANTE) return "rouge";
  if (fg > SEUIL_DOMINANTE) return "vert";
  if (fb > SEUIL_DOMINANTE) return "bleu";
  return k.lum > SEUIL_BLANC ? "blanc" : "gris";
}

void lireCouleur(CapteurCouleur &k) {
  if (!k.present) return;
  choisirCanal(k.canal);

  uint8_t statut = 0;
  if (!tcsLire(0x13, &statut, 1) || !(statut & 0x01)) return;  // AVALID

  uint8_t d[8];
  if (!tcsLire(0x14, d, 8)) return;
  k.c = d[0] | (d[1] << 8);
  k.r = d[2] | (d[3] << 8);
  k.g = d[4] | (d[5] << 8);
  k.b = d[6] | (d[7] << 8);
  k.valide = true;

  uint16_t maxi = comptageMaxTcs();
  k.lum = (float) k.c / maxi;
  k.sature = k.c >= maxi;

  // Lux et temperature de couleur : methode AMS DN40
  float ir = (k.r + k.g + k.b > k.c) ? (k.r + k.g + k.b - k.c) / 2.0 : 0;
  float r2 = k.r - ir, g2 = k.g - ir, b2 = k.b - ir;
  float atimeMs = (256 - TCS_ATIME) * 2.4;
  float cpl = atimeMs * TCS_GAINS[TCS_GAIN] / 310.0;
  k.lux = (0.136 * r2 + g2 - 0.444 * b2) / cpl;
  if (k.lux < 0) k.lux = 0;
  k.cct = (r2 > 0) ? (int) (3810.0 * b2 / r2 + 1391) : 0;

  k.couleur = classerCouleur(k);
}

// ================== SORTIES ==================

void afficherTexte() {
  char buf[40];
  Serial.println();
  Serial.printf("-------- t=%.1f s   boucle %lu Hz --------\n",
                millis() / 1000.0, boucleHz);

  Serial.print("DISTANCE ");
  for (int i = 0; i < 4; i++) {
    texteDistance(tof[i], buf, sizeof(buf));
    Serial.printf("  %s %-10s", tof[i].nom, buf);
  }
  Serial.println();

  for (int i = 0; i < 3; i++) {
    const CapteurCouleur &k = couleurs[i];
    Serial.printf("COULEUR   %-10s ", k.nom);
    if (!k.present)      Serial.println("ABSENT");
    else if (!k.valide)  Serial.println("pas encore de mesure");
    else Serial.printf("R=%5u G=%5u B=%5u C=%5u  lum=%.2f  %5.0f lux  %5d K  -> %s\n",
                       k.r, k.g, k.b, k.c, k.lum, k.lux, k.cct, k.couleur);
  }

  if (imuPresent) {
    Serial.printf("GYRO      cap=%7.1f deg   gyro x=%6.2f y=%6.2f z=%6.2f deg/s\n",
                  cap, gyro.gyroX, gyro.gyroY, gyro.gyroZ - biaisGyroZ);
    Serial.printf("ACCEL     x=%5.2f y=%5.2f z=%5.2f g   temp=%.1f C\n",
                  accel.accelX, accel.accelY, accel.accelZ, temperatureImu);
  } else {
    Serial.println("GYRO      ABSENT");
  }
}

void envoyerJson() {
  Serial.printf("[DATA]{\"t\":%lu,\"hz\":%lu,\"tof\":[", millis(), boucleHz);
  for (int i = 0; i < 4; i++) {
    const CapteurToF &t = tof[i];
    Serial.printf("%s{\"n\":\"%s\",\"m\":\"%s\",\"d\":%d,\"s\":\"%s\",\"p\":%d",
                  i ? "," : "", t.nom, t.modele, t.brut, codeEtat(t), t.porteeMm);
    if (i == 0 && t.present)
      Serial.printf(",\"st\":\"%s\"",
                    VL53L1X::rangeStatusToString((VL53L1X::RangeStatus) t.statutL1X));
    Serial.print("}");
  }
  Serial.print("],\"col\":[");
  for (int i = 0; i < 3; i++) {
    const CapteurCouleur &k = couleurs[i];
    Serial.printf("%s{\"n\":\"%s\",\"ok\":%d", i ? "," : "", k.nom,
                  k.present ? (k.valide ? 1 : 0) : -1);
    if (k.valide)
      Serial.printf(",\"r\":%u,\"g\":%u,\"b\":%u,\"c\":%u,\"lum\":%.3f,"
                    "\"lux\":%.0f,\"k\":%d,\"sat\":%d,\"nom\":\"%s\"",
                    k.r, k.g, k.b, k.c, k.lum, k.lux, k.cct, k.sature, k.couleur);
    Serial.print("}");
  }
  Serial.print("],\"imu\":");
  if (imuPresent)
    Serial.printf("{\"cap\":%.2f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
                  "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"temp\":%.1f,\"biais\":%.4f}",
                  cap, gyro.gyroX, gyro.gyroY, gyro.gyroZ - biaisGyroZ,
                  accel.accelX, accel.accelY, accel.accelZ, temperatureImu, biaisGyroZ);
  else
    Serial.print("null");
  Serial.println("}");
}

void lireCommandes() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'c' || ch == 'C') calibrerGyro();
    else if (ch == 'z' || ch == 'Z') { cap = 0; Serial.println("Cap remis a zero"); }
  }
}

// ================== PROGRAMME ==================

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Moteurs coupes : le rover ne doit pas bouger pendant ce test
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);
  for (uint8_t p : BROCHES_MOTEURS) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(100);

  Serial.println();
  Serial.println("================================================");
  Serial.println("   TEST CAPTEURS : DISTANCE + COULEUR + GYRO");
  Serial.println("   moteurs coupes - commandes : c = calibrer, z = cap a 0");
  Serial.println("================================================");

  muxPresent = repond(ADRESSE_MUX);
  Serial.printf("  TCA9548A 0x73          %s\n", muxPresent ? "OK" : "ABSENT");

  if (muxPresent) {
    choisirCanal(0);
    tofAvant.setBus(&Wire);
    tofAvant.setTimeout(200);
    if (tofAvant.init()) {
      tofAvant.setDistanceMode(VL53L1X::Long);
      tofAvant.setMeasurementTimingBudget(50000);
      tofAvant.startContinuous(50);
      tof[0].present = true;
    }
    initVL53L0X(tofArriere, 1);
    initVL53L0X(tofDroite, 2);
    initVL53L0X(tofGauche, 3);
    for (auto &k : couleurs) initCouleur(k);
  }

  for (int i = 0; i < 4; i++)
    Serial.printf("  canal %d  %-8s %s  %s\n", i, tof[i].nom, tof[i].modele,
                  !tof[i].present ? "ABSENT"
                  : tof[i].fiable ? "OK" : "present, NON FIABLE");
  for (auto &k : couleurs)
    Serial.printf("  canal %d  %-10s TCS34725  %s\n", k.canal, k.nom,
                  k.present ? "OK" : "ABSENT");

  // Mux ferme avant le gyroscope, qui est sur le bus principal
  if (muxPresent) {
    Wire.beginTransmission(ADRESSE_MUX);
    Wire.write(0);
    Wire.endTransmission();
  }
  if (IMU.init(calib, ADRESSE_IMU) == 0) {
    imuPresent = true;
    IMU.setGyroRange(250);
    Serial.println("  MPU-6500 0x68          OK (+/-250 deg/s)");
    delay(300);
    calibrerGyro();
  } else {
    Serial.println("  MPU-6500 0x68          ABSENT");
  }
  Serial.println("================================================");

  dernierEchantillonGyro = micros();
  dernierCalculHz = millis();
}

void loop() {
  nbBoucles++;
  lireCommandes();

  // Le gyroscope est relu entre chaque capteur : le cap s'integre
  // ainsi par petits pas meme si une lecture de distance attend.
  mettreAJourIMU();
  for (uint8_t i = 0; i < 4; i++) {
    lireDistance(i);
    mettreAJourIMU();
  }
  for (auto &k : couleurs) lireCouleur(k);
  mettreAJourIMU();

  unsigned long maintenant = millis();
  if (maintenant - dernierCalculHz >= 1000) {
    boucleHz = nbBoucles * 1000UL / (maintenant - dernierCalculHz);
    nbBoucles = 0;
    dernierCalculHz = maintenant;
  }
  if (PERIODE_JSON_MS && maintenant - dernierJson >= PERIODE_JSON_MS) {
    dernierJson = maintenant;
    envoyerJson();
  }
  if (PERIODE_TEXTE_MS && maintenant - dernierTexte >= PERIODE_TEXTE_MS) {
    dernierTexte = maintenant;
    afficherTexte();
  }
}
