/*
 * 13_EVITEMENT_4_CAPTEURS.cpp
 * ------------------------------------------------------------
 * EVITEMENT D'OBSTACLES - 4 CAPTEURS, MODE AUTONOME
 *
 * NOUVEAUTE PAR RAPPORT AU PROGRAMME 12 :
 * Le capteur ARRIERE est utilise. Jusqu'ici la marche arriere se
 * faisait a l'aveugle : le rover reculait de 12 cm sans savoir ce
 * qu'il y avait derriere lui. Desormais :
 *   - il verifie l'espace disponible AVANT de reculer
 *   - il surveille en permanence PENDANT le recul
 *   - il s'arrete des qu'un obstacle arriere approche
 *   - si l'espace est insuffisant, il tourne sur place sans reculer
 *
 * BIBLIOTHEQUES : VL53L1X (Pololu), VL53L0X (Pololu), FastIMU
 *
 * REFERENCES MATERIEL (brochage et etat des composants) :
 *   - Rover_Cablage_Moteurs.pdf (01.09.26)
 *   - 01_Rover_Suivi.xlsx : feuilles Suivi, Connexions, Calibration
 *
 * >>> ARRET : COUPER L'INTERRUPTEUR. <<<
 * Rester a portee du rover pendant tout l'essai.
 * ------------------------------------------------------------
 */

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <VL53L0X.h>
#include "FastIMU.h"

// ================== BROCHAGE ==================
// Conforme a Rover_Cablage_Moteurs.pdf et a la feuille Connexions.

#define SDA_PIN 38
#define SCL_PIN 39
#define I2C_CLOCK_HZ 100000

#define ADRESSE_MUX 0x73        // TCA9548A, A0/A1 soudes sur VCC
#define ADRESSE_IMU 0x68        // MPU-6500, AD0 a la masse
#define WHO_AM_I_REG 0x75
#define WHO_AM_I_MPU6500 0x70

#define CANAL_AVANT   0
#define CANAL_ARRIERE 1
#define CANAL_DROITE  2
#define CANAL_GAUCHE  3

#define STBY_PIN 16

// PAS DE LED DE CARTE : la LED RGB de la DevKitC-1 est sur GPIO48
// (v1.0) ou GPIO38 (v1.1). GPIO48 = encodeur M3 phase B et
// GPIO38 = SDA : l'utiliser fausserait l'encodeur ou le bus I2C.

#define M1_PWM 2
#define M1_IN1 4        // inverse
#define M1_IN2 1        // inverse
#define M3_PWM 5
#define M3_IN1 6
#define M3_IN2 7
#define M2_PWM 10
#define M2_IN1 11
#define M2_IN2 12
#define M4_PWM 13
#define M4_IN1 15       // inverse
#define M4_IN2 14       // inverse

#define ENC1_A 40
#define ENC1_B 41
#define ENC3_A 47
#define ENC3_B 48
#define ENC2_A 19       // inverse : positif = le rover avance
#define ENC2_B 18
#define ENC4_A 17       // inverse
#define ENC4_B 21

// ================== REGLAGES ==================

const int PWM_FREQ = 20000;
const int PWM_RES = 8;

const int VITESSE_AVANCE = 100;      // sur 255
const int VITESSE_ROTATION = 110;
const int VITESSE_RECUL = 100;

const int SEUIL_OBSTACLE_MM = 250;   // declenchement avant
const int SEUIL_ARRIERE_MM = 180;    // distance minimale derriere
const float RECUL_CM = 12.0;
const float ANGLE_VIRAGE_DEG = 70.0;

// Feuille Calibration, bloc 1
const float IMPULSIONS_PAR_TOUR = 576.0;
const float CIRCONFERENCE_CM = 6.7 * 3.14159265;
const float MM_PAR_IMPULSION = CIRCONFERENCE_CM * 10.0 / IMPULSIONS_PAR_TOUR;

// Le VL53L0X renvoie 8190/8191 quand il n'a pas de mesure :
// c'est un code d'erreur, PAS une distance (Calibration, bloc 5).
const int VL53L0X_CODE_ERREUR = 8190;

const int COMPTE_A_REBOURS_S = 10;
const int DUREE_MAX_MIN = 5;         // 0 pour desactiver
const unsigned long DEBUG_PERIODE_MS = 500;  // 0 pour desactiver

// ================== STRUCTURES ==================

struct Moteur {
  int pwm;
  int in1;
  int in2;
  int canal;
  const char* nom;
};

struct Encodeur {
  uint8_t pinA;
  uint8_t pinB;
  volatile long compte;
  volatile uint8_t etatPrec;
};

enum Etat {
  E_AVANCE,
  E_ARRET_COMPLET,
  E_VERIFIE_ARRIERE,
  E_RECULE,
  E_MESURE_COTES,
  E_TOURNE,
  E_FIN
};

enum EtatMesure {
  M_ABSENT,        // capteur non detecte au demarrage
  M_JAMAIS_LU,
  M_OK,
  M_HORS_PORTEE,   // VL53L0X : code 8190/8191, rien dans sa portee
  M_TIMEOUT
};

// Un capteur de distance par canal du multiplexeur (index = canal).
struct CapteurToF {
  const char* nom;
  const char* modele;
  int porteeMm;       // portee reelle mesuree (Calibration, bloc 5)
  bool fiable;        // false : lu et journalise, ignore pour decider
  bool present;
  int brut;           // derniere valeur renvoyee par la bibliotheque
  uint8_t statutL1X;  // VL53L1X uniquement : range_status
  EtatMesure etat;
};

// ================== OBJETS ==================

Moteur moteurs[4] = {
  { M1_PWM, M1_IN1, M1_IN2, 0, "M1 avant-droit" },
  { M3_PWM, M3_IN1, M3_IN2, 1, "M3 arriere-droit" },
  { M2_PWM, M2_IN1, M2_IN2, 2, "M2 avant-gauche" },
  { M4_PWM, M4_IN1, M4_IN2, 3, "M4 arriere-gauche" }
};

// Meme ordre que moteurs[]
Encodeur encodeurs[4] = {
  { ENC1_A, ENC1_B, 0, 0 },
  { ENC3_A, ENC3_B, 0, 0 },
  { ENC2_A, ENC2_B, 0, 0 },
  { ENC4_A, ENC4_B, 0, 0 }
};

static const int8_t TABLE_QUAD[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

VL53L1X tofAvant;
VL53L0X tofArriere;
VL53L0X tofDroite;
VL53L0X tofGauche;

// Etat au 01.09.26 (Suivi n.52 et 54) : le VL53L0X droit est
// DEFAILLANT (renvoie 8191, ne reagit qu'a moins de 10,5 cm).
// Passer fiable a true une fois remplace par celui de reserve.
CapteurToF capteurs[4] = {
  { "avant",   "VL53L1X", 4000, true,  false, -1, 0, M_JAMAIS_LU },
  { "arriere", "VL53L0X", 1200, true,  false, -1, 0, M_JAMAIS_LU },
  { "droite",  "VL53L0X",    0, false, false, -1, 0, M_JAMAIS_LU },
  { "gauche",  "VL53L0X",  600, true,  false, -1, 0, M_JAMAIS_LU }
};

MPU6500 IMU;
calData calib = { 0 };
GyroData gyroData;
bool imuPresent = false;

// ================== ETAT GLOBAL ==================

Etat etat = E_AVANCE;
float biaisGyroZ = 0.0;
float cap = 0.0;
float capCible = 0.0;
unsigned long dernierEchantillonGyro = 0;
unsigned long horodatageEtat = 0;
unsigned long horodatageDepart = 0;
bool virageADroite = true;
bool reculPossible = true;
int nbObstacles = 0;
int nbReculsAnnules = 0;

unsigned long dernierDebug = 0;
unsigned long nbBoucles = 0;
long comptesDebugPrec[4] = { 0, 0, 0, 0 };

// ================== INTERRUPTION ENCODEURS ==================

void IRAM_ATTR isrEncodeur(void* arg) {
  Encodeur* e = (Encodeur*) arg;
  uint8_t s = (digitalRead(e->pinA) << 1) | digitalRead(e->pinB);
  e->compte += TABLE_QUAD[(e->etatPrec << 2) | s];
  e->etatPrec = s;
}

long compteMoyen() {
  long total = 0;
  noInterrupts();
  for (int i = 0; i < 4; i++) total += abs(encodeurs[i].compte);
  interrupts();
  return total / 4;
}

void remettreCompteursAZero() {
  noInterrupts();
  for (int i = 0; i < 4; i++) encodeurs[i].compte = 0;
  interrupts();
  for (int i = 0; i < 4; i++) comptesDebugPrec[i] = 0;
}

// ================== MULTIPLEXEUR ==================

void choisirCanal(uint8_t canal) {
  Wire.beginTransmission(ADRESSE_MUX);
  Wire.write(1 << canal);
  Wire.endTransmission();
}

void fermerCanaux() {
  Wire.beginTransmission(ADRESSE_MUX);
  Wire.write(0);
  Wire.endTransmission();
}

// ================== PWM ET MOTEURS ==================

void pwmInit(const Moteur &m) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(m.pwm, PWM_FREQ, PWM_RES);
#else
  ledcSetup(m.canal, PWM_FREQ, PWM_RES);
  ledcAttachPin(m.pwm, m.canal);
#endif
}

void pwmEcrire(const Moteur &m, int v) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(m.pwm, v);
#else
  ledcWrite(m.canal, v);
#endif
}

void moteurAvant(const Moteur &m, int v) {
  digitalWrite(m.in1, HIGH);
  digitalWrite(m.in2, LOW);
  pwmEcrire(m, v);
}

void moteurArriere(const Moteur &m, int v) {
  digitalWrite(m.in1, LOW);
  digitalWrite(m.in2, HIGH);
  pwmEcrire(m, v);
}

void arreterTout() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(moteurs[i].in1, LOW);
    digitalWrite(moteurs[i].in2, LOW);
    pwmEcrire(moteurs[i], 0);
  }
}

void avancer(int v) {
  for (int i = 0; i < 4; i++) moteurAvant(moteurs[i], v);
}

void reculer(int v) {
  for (int i = 0; i < 4; i++) moteurArriere(moteurs[i], v);
}

void tournerDroite(int v) {
  moteurArriere(moteurs[0], v);
  moteurArriere(moteurs[1], v);
  moteurAvant(moteurs[2], v);
  moteurAvant(moteurs[3], v);
}

void tournerGauche(int v) {
  moteurAvant(moteurs[0], v);
  moteurAvant(moteurs[1], v);
  moteurArriere(moteurs[2], v);
  moteurArriere(moteurs[3], v);
}

// ================== CAPTEURS ==================

// Lit le capteur du canal donne et memorise le resultat pour le debug.
// Renvoie la distance utilisable pour decider, en mm :
//   - la mesure si elle est valide
//   - la portee du capteur si rien n'est dans sa portee (8190/8191)
//   - -1 si absent, timeout ou capteur non fiable
int lireCapteur(uint8_t canal) {
  CapteurToF &c = capteurs[canal];
  if (!c.present) {
    c.etat = M_ABSENT;
    return -1;
  }

  choisirCanal(canal);
  int d;
  bool timeout;
  if (canal == CANAL_AVANT) {
    d = tofAvant.read();
    timeout = tofAvant.timeoutOccurred();
    c.statutL1X = tofAvant.ranging_data.range_status;
  } else {
    VL53L0X &s = (canal == CANAL_ARRIERE) ? tofArriere
               : (canal == CANAL_DROITE)  ? tofDroite : tofGauche;
    d = s.readRangeContinuousMillimeters();
    timeout = s.timeoutOccurred();
  }

  c.brut = d;
  if (timeout) {
    c.etat = M_TIMEOUT;
    return -1;
  }
  if (canal != CANAL_AVANT && d >= VL53L0X_CODE_ERREUR) {
    c.etat = M_HORS_PORTEE;
    return c.fiable ? c.porteeMm : -1;
  }
  c.etat = M_OK;
  return c.fiable ? d : -1;
}

int lireAvant()   { return lireCapteur(CANAL_AVANT); }
int lireArriere() { return lireCapteur(CANAL_ARRIERE); }

// Texte court de la derniere mesure : "812", ">1200", "ERR", "ABS",
// "--" (jamais lu), prefixe "NF:" si le capteur est non fiable.
void texteMesure(uint8_t canal, char* buf, size_t taille) {
  const CapteurToF &c = capteurs[canal];
  const char* nf = c.fiable ? "" : "NF:";
  switch (c.etat) {
    case M_ABSENT:      snprintf(buf, taille, "ABS"); break;
    case M_JAMAIS_LU:   snprintf(buf, taille, "%s--", nf); break;
    case M_TIMEOUT:     snprintf(buf, taille, "%sERR", nf); break;
    case M_HORS_PORTEE:
      if (c.fiable) snprintf(buf, taille, ">%d", c.porteeMm);
      else          snprintf(buf, taille, "NF:%d", c.brut);
      break;
    case M_OK:
      if (canal == CANAL_AVANT && c.statutL1X != VL53L1X::RangeValid)
        snprintf(buf, taille, "%s%d(%s)", nf, c.brut,
                 VL53L1X::rangeStatusToString((VL53L1X::RangeStatus) c.statutL1X));
      else
        snprintf(buf, taille, "%s%d", nf, c.brut);
      break;
  }
}

void imprimerMesure(uint8_t canal) {
  char buf[64];
  texteMesure(canal, buf, sizeof(buf));
  Serial.print("  ");
  Serial.print(capteurs[canal].nom);
  Serial.print(" = ");
  Serial.print(buf);
  if (capteurs[canal].etat == M_OK || capteurs[canal].etat == M_HORS_PORTEE)
    Serial.print(" mm");
  Serial.println();
}

// ================== GYROSCOPE ==================

float lireVitesseZ() {
  IMU.update();
  IMU.getGyro(&gyroData);
  return gyroData.gyroZ;
}

void mettreAJourCap() {
  unsigned long maintenant = micros();
  float dt = (maintenant - dernierEchantillonGyro) / 1000000.0;
  dernierEchantillonGyro = maintenant;
  if (!imuPresent) return;
  cap += (lireVitesseZ() - biaisGyroZ) * dt;
}

void calibrerGyro() {
  Serial.println("Calibration du gyroscope - NE PAS BOUGER LE ROVER...");

  double somme = 0.0;
  for (int i = 0; i < 1500; i++) {
    somme += lireVitesseZ();
    delay(1);
  }

  biaisGyroZ = somme / 1500.0;
  Serial.print("Biais axe Z : ");
  Serial.print(biaisGyroZ, 4);
  Serial.println(" deg/s");
  Serial.println();
  dernierEchantillonGyro = micros();
}

// ================== INVENTAIRE MATERIEL ==================

// Plan d'adressage I2C (Suivi n.19). installe = "Fait" dans le suivi.
struct PeripheriqueI2C {
  uint8_t adresse;
  const char* nom;
  bool installe;
};

const PeripheriqueI2C PLAN_I2C[] = {
  { 0x20, "MCP23017",          false },
  { 0x38, "DHT20",             false },
  { 0x3C, "OLED",              false },
  { 0x40, "PCA9685 #1 servos", false },
  { 0x41, "PCA9685 #2 LED",    false },
  { 0x48, "ADS1115",           false },
  { 0x50, "ESP32-S3-CAM",      false },
  { 0x68, "MPU-6500",          true  },
  { ADRESSE_MUX, "TCA9548A",   true  },
};
const int NB_PLAN_I2C = sizeof(PLAN_I2C) / sizeof(PLAN_I2C[0]);

// Contenu prevu de chaque canal du multiplexeur (Connexions n.49-55).
const char* PLAN_CANAUX[8] = {
  "VL53L1X avant", "VL53L0X arriere", "VL53L0X droite", "VL53L0X gauche",
  "TCS34725 sol gauche (prevu)", "TCS34725 sol centre (prevu)",
  "TCS34725 sol droit (prevu)", "libre"
};

bool repond(uint8_t adresse) {
  Wire.beginTransmission(adresse);
  return Wire.endTransmission() == 0;
}

const PeripheriqueI2C* chercherPlan(uint8_t adresse) {
  for (int i = 0; i < NB_PLAN_I2C; i++)
    if (PLAN_I2C[i].adresse == adresse) return &PLAN_I2C[i];
  return nullptr;
}

bool lireRegistre(uint8_t adresse, uint8_t reg, uint8_t &valeur) {
  Wire.beginTransmission(adresse);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(adresse, (uint8_t) 1) != 1) return false;
  valeur = Wire.read();
  return true;
}

void scannerBusPrincipal(bool &muxPresent) {
  Serial.println("--- Bus I2C principal (SDA=GPIO38, SCL=GPIO39) ---");

  // Canaux du mux fermes pour ne voir que le bus principal
  muxPresent = repond(ADRESSE_MUX);
  if (muxPresent) fermerCanaux();

  bool vu[128] = { false };
  for (uint8_t a = 1; a < 127; a++) {
    if (!repond(a)) continue;
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
}

void scannerCanauxMux(bool muxPresent) {
  Serial.println("--- Canaux du TCA9548A (0x73) ---");
  if (!muxPresent) {
    Serial.println("  multiplexeur absent : aucun capteur de distance joignable");
    return;
  }

  // Ce qui repond deja canaux fermes appartient au bus principal
  fermerCanaux();
  bool principal[128] = { false };
  for (uint8_t a = 1; a < 127; a++) principal[a] = repond(a);

  for (uint8_t canal = 0; canal < 8; canal++) {
    choisirCanal(canal);
    Serial.printf("  canal %d  %-28s :", canal, PLAN_CANAUX[canal]);
    int n = 0;
    for (uint8_t a = 1; a < 127; a++) {
      if (principal[a] || a == ADRESSE_MUX) continue;
      if (repond(a)) {
        Serial.printf(" 0x%02X", a);
        n++;
      }
    }
    Serial.println(n ? "" : " vide");
  }
  fermerCanaux();
}

void verifierIMU() {
  uint8_t who = 0;
  Serial.println("--- Gyroscope ---");
  if (!lireRegistre(ADRESSE_IMU, WHO_AM_I_REG, who)) {
    Serial.println("  MPU-6500 (0x68)      ne repond pas");
    return;
  }
  Serial.printf("  WHO_AM_I = 0x%02X  %s\n", who,
                who == WHO_AM_I_MPU6500 ? "(MPU-6500 confirme)"
                                        : "<- attendu 0x70 pour un MPU-6500");
}

void afficherCablageFixe() {
  Serial.println("--- Cablage (non detectable, d'apres le plan) ---");
  for (int i = 0; i < 4; i++) {
    const Moteur &m = moteurs[i];
    Serial.printf("  %-18s PWM=GPIO%-2d IN1=GPIO%-2d IN2=GPIO%-2d  enc A=GPIO%d B=GPIO%d\n",
                  m.nom, m.pwm, m.in1, m.in2,
                  encodeurs[i].pinA, encodeurs[i].pinB);
  }
  Serial.println("  STBY=GPIO16 (2 x TB6612FNG)  - droit #1 : M1/M3, gauche #2 : M2/M4");
  Serial.println("  Inversions logicielles : IN1/IN2 de M1 et M4, phases A/B de M2 et M4");
  Serial.printf("  Encodeurs : %.0f impulsions/tour, %.3f mm/impulsion\n",
                IMPULSIONS_PAR_TOUR, MM_PAR_IMPULSION);

  Serial.print("  Niveaux encodeurs au repos (A,B) :");
  for (int i = 0; i < 4; i++)
    Serial.printf(" %d,%d", digitalRead(encodeurs[i].pinA),
                  digitalRead(encodeurs[i].pinB));
  Serial.println();
}

void afficherCapteurs() {
  Serial.println("--- Capteurs de distance ---");
  for (int i = 0; i < 4; i++) {
    const CapteurToF &c = capteurs[i];
    Serial.printf("  canal %d  %-8s %s  ", i, c.nom, c.modele);
    if (!c.present)  Serial.println("ABSENT");
    else if (!c.fiable) Serial.println("present, NON FIABLE (ignore pour decider)");
    else Serial.printf("OK, portee %d mm\n", c.porteeMm);
  }
}

// ================== ARRET ==================

void arretDefinitif(const char* raison) {
  arreterTout();
  digitalWrite(STBY_PIN, LOW);
  etat = E_FIN;

  Serial.println();
  Serial.print(">>> ARRET : ");
  Serial.println(raison);
  Serial.print("    Obstacles evites : ");
  Serial.println(nbObstacles);
  Serial.print("    Reculs annules (pas de place) : ");
  Serial.println(nbReculsAnnules);
}

// Prepare la rotation : mesure les cotes, choisit la direction,
// fixe le cap cible et lance le mouvement.
void lancerRotation() {
  int dGauche = lireCapteur(CANAL_GAUCHE);
  int dDroite = lireCapteur(CANAL_DROITE);

  imprimerMesure(CANAL_GAUCHE);
  imprimerMesure(CANAL_DROITE);

  if (dGauche > 0 && dDroite > 0) {
    virageADroite = (dDroite > dGauche);
  } else {
    // Sans mesure laterale fiable, on alterne pour eviter de
    // tourner toujours du meme cote et de boucler sur place.
    virageADroite = !virageADroite;
  }

  // Convention validee : rotation a DROITE = cap NEGATIF
  capCible = cap + (virageADroite ? -ANGLE_VIRAGE_DEG : ANGLE_VIRAGE_DEG);

  if (virageADroite) {
    tournerDroite(VITESSE_ROTATION);
    Serial.println("  rotation a droite");
  } else {
    tournerGauche(VITESSE_ROTATION);
    Serial.println("  rotation a gauche");
  }

  etat = E_TOURNE;
  horodatageEtat = millis();
}

// ================== DEBUG PERIODIQUE ==================

const char* nomEtat(Etat e) {
  switch (e) {
    case E_AVANCE:          return "AVANCE";
    case E_ARRET_COMPLET:   return "ARRET";
    case E_VERIFIE_ARRIERE: return "VERIF_AR";
    case E_RECULE:          return "RECULE";
    case E_MESURE_COTES:    return "MESURE";
    case E_TOURNE:          return "TOURNE";
    case E_FIN:             return "FIN";
  }
  return "?";
}

// Une ligne par periode, format cle=valeur.
// Distances en mm, derniere valeur lue : ">1200" = rien dans la
// portee, ERR = timeout, ABS = absent, -- = pas encore lu,
// NF: = capteur non fiable (valeur brute, ignoree pour decider).
// Vitesses par roue en mm/s, positif = vers l'avant.
void envoyerDebug() {
  if (DEBUG_PERIODE_MS == 0) return;
  unsigned long maintenant = millis();
  if (maintenant - dernierDebug < DEBUG_PERIODE_MS) return;
  unsigned long ecart = maintenant - dernierDebug;
  dernierDebug = maintenant;

  long comptes[4];
  noInterrupts();
  for (int i = 0; i < 4; i++) comptes[i] = encodeurs[i].compte;
  interrupts();

  int vitesse[4];
  for (int i = 0; i < 4; i++) {
    vitesse[i] = (comptes[i] - comptesDebugPrec[i]) * MM_PAR_IMPULSION * 1000.0 / ecart;
    comptesDebugPrec[i] = comptes[i];
  }

  char av[64], ar[16], dr[16], ga[16];
  texteMesure(CANAL_AVANT, av, sizeof(av));
  texteMesure(CANAL_ARRIERE, ar, sizeof(ar));
  texteMesure(CANAL_DROITE, dr, sizeof(dr));
  texteMesure(CANAL_GAUCHE, ga, sizeof(ga));

  Serial.printf(
      "[DBG] t=%lu etat=%s av=%s ar=%s dr=%s ga=%s cap=%.1f cible=%.1f "
      "v_M1=%d v_M3=%d v_M2=%d v_M4=%d enc=%ld,%ld,%ld,%ld "
      "obst=%d annul=%d boucle_hz=%lu\n",
      maintenant / 1000, nomEtat(etat), av, ar, dr, ga,
      cap, capCible,
      vitesse[0], vitesse[1], vitesse[2], vitesse[3],
      comptes[0], comptes[1], comptes[2], comptes[3],
      nbObstacles, nbReculsAnnules,
      nbBoucles * 1000UL / ecart);
  nbBoucles = 0;
}

// ================== PROGRAMME ==================

void initVL53L0X(VL53L0X &capteur, uint8_t canal) {
  choisirCanal(canal);
  capteur.setBus(&Wire);
  capteur.setTimeout(200);
  if (capteur.init()) {
    capteur.startContinuous();
    capteurs[canal].present = true;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- Moteurs ---
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);
  for (int i = 0; i < 4; i++) {
    pinMode(moteurs[i].in1, OUTPUT);
    pinMode(moteurs[i].in2, OUTPUT);
    pwmInit(moteurs[i]);
  }
  arreterTout();

  // --- Encodeurs ---
  for (int i = 0; i < 4; i++) {
    pinMode(encodeurs[i].pinA, INPUT);
    pinMode(encodeurs[i].pinB, INPUT);
    encodeurs[i].etatPrec =
        (digitalRead(encodeurs[i].pinA) << 1) | digitalRead(encodeurs[i].pinB);
    attachInterruptArg(digitalPinToInterrupt(encodeurs[i].pinA),
                       isrEncodeur, &encodeurs[i], CHANGE);
    attachInterruptArg(digitalPinToInterrupt(encodeurs[i].pinB),
                       isrEncodeur, &encodeurs[i], CHANGE);
  }

  // --- Bus I2C ---
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(100);

  Serial.println();
  Serial.println("================================================");
  Serial.println("   EVITEMENT - 4 CAPTEURS - MODE AUTONOME");
  Serial.println("================================================");
  Serial.println();
  Serial.println("=========== INVENTAIRE MATERIEL ===========");

  bool muxPresent = false;
  scannerBusPrincipal(muxPresent);
  scannerCanauxMux(muxPresent);
  verifierIMU();
  afficherCablageFixe();

  // --- Capteurs de distance ---
  choisirCanal(CANAL_AVANT);
  tofAvant.setBus(&Wire);
  tofAvant.setTimeout(200);
  if (tofAvant.init()) {
    tofAvant.setDistanceMode(VL53L1X::Long);
    tofAvant.setMeasurementTimingBudget(50000);
    tofAvant.startContinuous(50);
    capteurs[CANAL_AVANT].present = true;
  }
  initVL53L0X(tofArriere, CANAL_ARRIERE);
  initVL53L0X(tofDroite, CANAL_DROITE);
  initVL53L0X(tofGauche, CANAL_GAUCHE);
  afficherCapteurs();

  // Premiere mesure de chaque capteur, pour verifier qu'ils repondent
  Serial.println("--- Premiere mesure ---");
  for (uint8_t c = 0; c < 4; c++) {
    lireCapteur(c);
    imprimerMesure(c);
  }
  Serial.println("===========================================");
  Serial.println();

  if (!capteurs[CANAL_AVANT].present) {
    Serial.println("!! CAPTEUR AVANT ABSENT - DEMARRAGE ANNULE.");
    while (true) delay(1000);
  }

  if (!capteurs[CANAL_ARRIERE].present) {
    Serial.println("  Note : sans capteur arriere, le recul se fera");
    Serial.println("  a l'aveugle comme dans la version precedente.");
    Serial.println();
  }

  // --- Gyroscope ---
  if (IMU.init(calib, ADRESSE_IMU) != 0) {
    Serial.println("  MPU-6500 absent : virages moins precis.");
  } else {
    imuPresent = true;
    IMU.setGyroRange(250);
    Serial.println("  MPU-6500                    OK (+/-250 deg/s)");
    delay(300);
    calibrerGyro();
  }

  // --- Compte a rebours ---
  Serial.println("================================================");
  Serial.print("  DEPART DANS ");
  Serial.print(COMPTE_A_REBOURS_S);
  Serial.println(" SECONDES");
  Serial.println("  Poser le rover au sol et s'ecarter.");
  Serial.println("  ARRET : COUPER L'INTERRUPTEUR.");
  Serial.println("================================================");

  for (int s = COMPTE_A_REBOURS_S; s > 0; s--) {
    Serial.print("  ");
    Serial.println(s);
    delay(1000);
  }

  Serial.println();
  Serial.println(">>> DEPART");
  Serial.println();

  digitalWrite(STBY_PIN, HIGH);
  remettreCompteursAZero();
  horodatageDepart = millis();
  horodatageEtat = millis();
  dernierEchantillonGyro = micros();
  dernierDebug = millis();
  etat = E_AVANCE;
  avancer(VITESSE_AVANCE);
}

void loop() {
  nbBoucles++;
  envoyerDebug();

  if (etat == E_FIN) {
    delay(500);
    return;
  }

  mettreAJourCap();

  // --- Duree maximale ---
  if (DUREE_MAX_MIN > 0 &&
      millis() - horodatageDepart > (unsigned long) DUREE_MAX_MIN * 60000UL) {
    arretDefinitif("duree maximale atteinte");
    return;
  }

  int distanceAvant = lireAvant();
  if (distanceAvant < 0) {
    arretDefinitif("le capteur avant ne repond plus");
    return;
  }

  switch (etat) {

    case E_AVANCE:
      if (distanceAvant < SEUIL_OBSTACLE_MM) {
        nbObstacles++;
        Serial.print("Obstacle #");
        Serial.print(nbObstacles);
        Serial.print(" a ");
        Serial.print(distanceAvant);
        Serial.println(" mm");

        arreterTout();
        etat = E_ARRET_COMPLET;
        horodatageEtat = millis();
      }
      break;

    case E_ARRET_COMPLET:
      // Laisse les roues s'immobiliser avant toute mesure aux
      // encodeurs : l'inertie fausserait le point de depart.
      if (millis() - horodatageEtat > 300) {
        etat = E_VERIFIE_ARRIERE;
        horodatageEtat = millis();
      }
      break;

    case E_VERIFIE_ARRIERE: {
      // NOUVEAU : on regarde derriere AVANT de reculer.
      int dArriere = lireArriere();

      if (!capteurs[CANAL_ARRIERE].present) {
        // Pas de capteur : comportement de l'ancienne version
        reculPossible = true;
      } else if (dArriere < 0) {
        // Lecture invalide : par prudence, on ne recule pas
        reculPossible = false;
        Serial.println("  arriere : lecture invalide, recul annule");
      } else {
        imprimerMesure(CANAL_ARRIERE);
        // Il faut la place de reculer, plus une marge de securite
        reculPossible = (dArriere > (RECUL_CM * 10 + SEUIL_ARRIERE_MM));
      }

      if (reculPossible) {
        remettreCompteursAZero();
        reculer(VITESSE_RECUL);
        etat = E_RECULE;
        horodatageEtat = millis();
      } else {
        // Pas la place : on tourne sur place, sans reculer.
        nbReculsAnnules++;
        Serial.println("  place insuffisante derriere -> rotation directe");
        lancerRotation();
      }
      break;
    }

    case E_RECULE: {
      // NOUVEAU : surveillance permanente pendant le recul.
      int dArriere = lireArriere();
      if (dArriere > 0 && dArriere < SEUIL_ARRIERE_MM) {
        arreterTout();
        Serial.print("  obstacle arriere a ");
        Serial.print(dArriere);
        Serial.println(" mm -> recul interrompu");
        delay(200);
        lancerRotation();
        break;
      }

      float parcouruCm = (compteMoyen() / IMPULSIONS_PAR_TOUR) * CIRCONFERENCE_CM;

      // Securite : si les encodeurs ne comptent pas, le recul ne
      // se terminerait jamais.
      if (parcouruCm >= RECUL_CM || millis() - horodatageEtat > 3000) {
        arreterTout();
        etat = E_MESURE_COTES;
        horodatageEtat = millis();
      }
      break;
    }

    case E_MESURE_COTES:
      if (millis() - horodatageEtat > 300) {
        lancerRotation();
      }
      break;

    case E_TOURNE: {
      bool termine = virageADroite ? (cap <= capCible) : (cap >= capCible);

      // Securite : sans gyroscope, la rotation ne finirait jamais.
      if (termine || (millis() - horodatageEtat > 4000)) {
        arreterTout();
        Serial.print("  cap : ");
        Serial.print(cap, 1);
        Serial.println(" deg");
        Serial.println();

        delay(200);
        remettreCompteursAZero();
        avancer(VITESSE_AVANCE);
        etat = E_AVANCE;
        horodatageEtat = millis();
      }
      break;
    }

    default:
      break;
  }
}

/*
 * ============================================================
 * CE QU'APPORTE LE CAPTEUR ARRIERE
 * ============================================================
 *
 * AVANT (programme 12)
 *   obstacle -> recul de 12 cm A L'AVEUGLE -> rotation
 *   Risque : reculer dans un mur, un pied de meuble, une marche.
 *
 * MAINTENANT
 *   obstacle -> mesure arriere
 *     espace suffisant  -> recul surveille en continu
 *     espace insuffisant-> rotation sur place, sans reculer
 *     obstacle detecte pendant le recul -> arret immediat, rotation
 *
 * Le rover peut donc se degager d'un coin sans se coincer :
 * c'etait la situation la plus problematique de la version 12.
 *
 * ============================================================
 * COMPTEURS AFFICHES A L'ARRET
 * ============================================================
 *   Obstacles evites  : nombre de detections frontales
 *   Reculs annules    : nombre de fois ou la place manquait derriere
 *
 * Un nombre eleve de reculs annules signale un environnement
 * encombre, ou un SEUIL_ARRIERE_MM trop prudent.
 *
 * ============================================================
 * REGLAGES
 * ============================================================
 *   SEUIL_ARRIERE_MM  180 mm : marge de securite derriere.
 *                     Diminuer si le rover refuse trop souvent de
 *                     reculer ; augmenter s'il frotte un mur.
 *   ANGLE_VIRAGE_DEG  70 deg. Augmenter s'il retrouve le meme
 *                     obstacle apres avoir tourne.
 *   capteurs[].fiable Passer a true le capteur droit une fois
 *                     remplace (Suivi n.54).
 *
 * ============================================================
 * LIMITE CONNUE
 * ============================================================
 * Le rover ne roule toujours pas droit : 4,5 % d'ecart entre les
 * cotes droit et gauche. La correction demande un asservissement
 * des encodeurs - etape suivante. Les vitesses v_M1..v_M4 du debug
 * permettent de mesurer cet ecart en roulant.
 * ============================================================
 */
