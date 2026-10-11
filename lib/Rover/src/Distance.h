/*
 * Distance.h - 4 capteurs de distance ToF derriere le TCA9548A
 * ------------------------------------------------------------
 *   canal 0 : VL53L1X avant      canal 1 : VL53L0X arriere
 *   canal 2 : VL53L0X droite     canal 3 : VL53L0X gauche
 *
 * Lecture non bloquante : distanceMettreAJour() ne lit un capteur que
 * s'il a une nouvelle mesure. Les dernieres valeurs restent dans tof[].
 */
#pragma once

#include <Arduino.h>

enum EtatMesure {
  M_ABSENT,        // capteur non detecte au demarrage
  M_JAMAIS_LU,
  M_OK,
  M_HORS_PORTEE,   // VL53L0X : code 8190/8191, rien dans sa portee
  M_TIMEOUT        // plus de nouvelle mesure depuis DELAI_MESURE_MAX_MS
};

struct CapteurToF {
  const char* nom;
  const char* modele;
  int porteeMm;       // portee reelle mesuree (feuille Calibration)
  bool fiable;        // false : lu et affiche, ignore pour decider
  bool present;
  int brut;           // derniere valeur renvoyee par la bibliotheque
  uint8_t statutL1X;  // VL53L1X uniquement : range_status
  EtatMesure etat;
  unsigned long derniereMs;
};

extern CapteurToF tof[4];

// VL53L0X : 8190/8191 = pas de mesure (code d'erreur, pas une distance)
constexpr int VL53L0X_CODE_ERREUR = 8190;
constexpr unsigned long DELAI_MESURE_MAX_MS = 500;

void distanceInit();          // le multiplexeur doit repondre
void distanceMettreAJour();   // a appeler a chaque tour de boucle

// Attend que chaque capteur present ait fourni une mesure (ou le delai)
void distanceAttendrePremiereMesure(unsigned long delaiMs);

// Distance utilisable pour decider, en mm :
//   la mesure si elle est valide, la portee du capteur si rien n'est
//   dans sa portee, -1 si absent, sans mesure ou non fiable.
int distanceUtile(uint8_t canal);

// Code court : ok, hp, err, abs, nf, --
const char* distanceCode(uint8_t canal);

// Texte de la derniere mesure.
//   lisible = true  : "812 mm", ">1200 mm", "ABSENT", "NF:8191", "ERR"
//   lisible = false : "812", ">1200", "ABS", "NF:8191", "ERR", "--",
//                     avec "(statut)" si le VL53L1X signale un defaut
void distanceTexte(uint8_t canal, char* buf, size_t taille, bool lisible);

const char* distanceStatutAvant();   // texte du range_status du VL53L1X
