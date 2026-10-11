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

#define SDA_PIN 38
#define SCL_PIN 39
#define I2C_CLOCK_HZ 100000

#define ADRESSE_MUX 0x73        // Mettre 0x70 si A0/A1 pas encore soudes
#define ADRESSE_IMU 0x68

#define CANAL_AVANT   0
#define CANAL_ARRIERE 1
#define CANAL_DROITE  2
#define CANAL_GAUCHE  3

#define STBY_PIN 16
#define LED_CARTE 48

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

const float IMPULSIONS_PAR_TOUR = 576.0;
const float CIRCONFERENCE_CM = 6.7 * 3.14159265;

const int COMPTE_A_REBOURS_S = 10;
const int DUREE_MAX_MIN = 5;         // 0 pour desactiver
const unsigned long DEBUG_PERIODE_MS = 500;  // 0 pour desactiver

// ================== STRUCTURES ==================

struct Moteur {
  int pwm;
  int in1;
  int in2;
  int canal;
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

// ================== OBJETS ==================

Moteur moteurs[4] = {
  { M1_PWM, M1_IN1, M1_IN2, 0 },   // avant-droit
  { M3_PWM, M3_IN1, M3_IN2, 1 },   // arriere-droit
  { M2_PWM, M2_IN1, M2_IN2, 2 },   // avant-gauche
  { M4_PWM, M4_IN1, M4_IN2, 3 }    // arriere-gauche
};

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

bool presentAvant = false;
bool presentArriere = false;
bool presentDroite = false;
bool presentGauche = false;

MPU6500 IMU;
calData calib = { 0 };
GyroData gyroData;

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

// Derniere distance lue par canal (-1 = jamais lue ou invalide).
// Le debug affiche ces valeurs au lieu de refaire des mesures.
int derniereDistance[4] = { -1, -1, -1, -1 };
unsigned long dernierDebug = 0;
unsigned long nbBoucles = 0;

// ================== TEMOIN LUMINEUX ==================

void couleurLed(uint8_t r, uint8_t v, uint8_t b) {
  // neopixelWrite : disponible sur les coeurs ESP32 2.x et 3.x.
  neopixelWrite(LED_CARTE, r, v, b);
}

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
}

// ================== MULTIPLEXEUR ==================

void choisirCanal(uint8_t canal) {
  Wire.beginTransmission(ADRESSE_MUX);
  Wire.write(1 << canal);
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

int lireAvant() {
  if (!presentAvant) return -1;
  choisirCanal(CANAL_AVANT);
  int d = tofAvant.read();
  if (tofAvant.timeoutOccurred()) d = -1;
  derniereDistance[CANAL_AVANT] = d;
  return d;
}

int lireVL53L0X(VL53L0X &capteur, uint8_t canal, bool present) {
  if (!present) return -1;
  choisirCanal(canal);
  int d = capteur.readRangeContinuousMillimeters();
  if (capteur.timeoutOccurred()) d = -1;
  derniereDistance[canal] = d;
  return d;
}

int lireArriere() {
  return lireVL53L0X(tofArriere, CANAL_ARRIERE, presentArriere);
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
  cap += (lireVitesseZ() - biaisGyroZ) * dt;
}

void calibrerGyro() {
  Serial.println("Calibration du gyroscope - NE PAS BOUGER LE ROVER...");

  double somme = 0.0;
  for (int i = 0; i < 1500; i++) {
    somme += lireVitesseZ();
    delay(1);
    if (i % 100 == 0) couleurLed(0, 0, (i / 100) % 2 ? 40 : 0);
  }

  biaisGyroZ = somme / 1500.0;
  Serial.print("Biais axe Z : ");
  Serial.print(biaisGyroZ, 4);
  Serial.println(" deg/s");
  Serial.println();
  dernierEchantillonGyro = micros();
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
  int dGauche = lireVL53L0X(tofGauche, CANAL_GAUCHE, presentGauche);
  int dDroite = lireVL53L0X(tofDroite, CANAL_DROITE, presentDroite);

  Serial.print("  gauche = "); Serial.print(dGauche);
  Serial.print(" mm   droite = "); Serial.print(dDroite);
  Serial.println(" mm");

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

// Une ligne par periode, format cle=valeur, facile a lire ou a parser.
// Distances : derniere valeur lue (-1 = absente/invalide/pas encore lue).
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

  Serial.printf(
      "[DBG] t=%lu etat=%s av=%d ar=%d dr=%d ga=%d cap=%.1f cible=%.1f "
      "enc=%ld,%ld,%ld,%ld obst=%d annul=%d boucle_hz=%lu\n",
      maintenant / 1000, nomEtat(etat),
      derniereDistance[CANAL_AVANT], derniereDistance[CANAL_ARRIERE],
      derniereDistance[CANAL_DROITE], derniereDistance[CANAL_GAUCHE],
      cap, capCible,
      comptes[0], comptes[1], comptes[2], comptes[3],
      nbObstacles, nbReculsAnnules,
      nbBoucles * 1000UL / ecart);
  nbBoucles = 0;
}

// ================== PROGRAMME ==================

void setup() {
  Serial.begin(115200);
  delay(1000);

  couleurLed(20, 20, 20);

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

  // --- Capteurs de distance ---
  choisirCanal(CANAL_AVANT);
  tofAvant.setBus(&Wire);
  tofAvant.setTimeout(200);
  if (tofAvant.init()) {
    tofAvant.setDistanceMode(VL53L1X::Long);
    tofAvant.setMeasurementTimingBudget(50000);
    tofAvant.startContinuous(50);
    presentAvant = true;
    Serial.println("  Canal 0 : VL53L1X avant     OK");
  } else {
    Serial.println("  Canal 0 : VL53L1X avant     ABSENT");
  }

  choisirCanal(CANAL_ARRIERE);
  tofArriere.setBus(&Wire);
  tofArriere.setTimeout(200);
  if (tofArriere.init()) {
    tofArriere.startContinuous();
    presentArriere = true;
    Serial.println("  Canal 1 : VL53L0X arriere   OK");
  } else {
    Serial.println("  Canal 1 : VL53L0X arriere   absent");
  }

  choisirCanal(CANAL_DROITE);
  tofDroite.setBus(&Wire);
  tofDroite.setTimeout(200);
  if (tofDroite.init()) {
    tofDroite.startContinuous();
    presentDroite = true;
    Serial.println("  Canal 2 : VL53L0X droite    OK");
  } else {
    Serial.println("  Canal 2 : VL53L0X droite    absent");
  }

  choisirCanal(CANAL_GAUCHE);
  tofGauche.setBus(&Wire);
  tofGauche.setTimeout(200);
  if (tofGauche.init()) {
    tofGauche.startContinuous();
    presentGauche = true;
    Serial.println("  Canal 3 : VL53L0X gauche    OK");
  } else {
    Serial.println("  Canal 3 : VL53L0X gauche    absent");
  }
  Serial.println();

  if (!presentAvant) {
    Serial.println("!! CAPTEUR AVANT ABSENT - DEMARRAGE ANNULE.");
    couleurLed(60, 0, 0);
    while (true) delay(1000);
  }

  if (!presentArriere) {
    Serial.println("  Note : sans capteur arriere, le recul se fera");
    Serial.println("  a l'aveugle comme dans la version precedente.");
    Serial.println();
  }

  // --- Gyroscope ---
  if (IMU.init(calib, ADRESSE_IMU) != 0) {
    Serial.println("  MPU-6500 absent : virages moins precis.");
  } else {
    IMU.setGyroRange(250);
    Serial.println("  MPU-6500                    OK");
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
    couleurLed(40, 30, 0);
    delay(500);
    couleurLed(0, 0, 0);
    delay(500);
  }

  Serial.println();
  Serial.println(">>> DEPART");
  Serial.println();

  couleurLed(0, 40, 0);

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
    couleurLed(60, 0, 0);
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
        couleurLed(40, 20, 0);
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

      if (!presentArriere) {
        // Pas de capteur : comportement de l'ancienne version
        reculPossible = true;
      } else if (dArriere < 0) {
        // Lecture invalide : par prudence, on ne recule pas
        reculPossible = false;
        Serial.println("  arriere : lecture invalide, recul annule");
      } else {
        Serial.print("  arriere = ");
        Serial.print(dArriere);
        Serial.println(" mm");
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
      if (presentArriere && dArriere > 0 && dArriere < SEUIL_ARRIERE_MM) {
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
        couleurLed(0, 40, 0);
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
 *
 * ============================================================
 * LIMITE CONNUE
 * ============================================================
 * Le rover ne roule toujours pas droit : 4,5 % d'ecart entre les
 * cotes droit et gauche. La correction demande un asservissement
 * des encodeurs - etape suivante.
 * ============================================================
 */
