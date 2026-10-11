/*
 * 13_EVITEMENT_4_CAPTEURS
 * ------------------------------------------------------------
 * EVITEMENT D'OBSTACLES - 4 CAPTEURS, MODE AUTONOME
 *
 * Le rover avance, s'arrete devant un obstacle, verifie la place
 * derriere avant de reculer, puis tourne vers le cote le plus degage.
 *
 * Fichiers :
 *   main.cpp       demarrage et boucle
 *   Evitement.cpp  machine a etats
 *   Debug.cpp      ligne [DBG] periodique
 *   Reglages.h     vitesses, seuils, durees
 *   lib/Rover/     materiel commun (moteurs, capteurs, gyro, bus)
 *
 * >>> ARRET : COUPER L'INTERRUPTEUR. <<<
 * Rester a portee du rover pendant tout l'essai.
 * ------------------------------------------------------------
 */

#include <Arduino.h>
#include "Bus.h"
#include "Config.h"
#include "Distance.h"
#include "Gyro.h"
#include "Inventaire.h"
#include "Moteurs.h"

#include "Debug.h"
#include "Evitement.h"
#include "Reglages.h"

void setup() {
  Serial.begin(VITESSE_SERIE);
  delay(1000);

  moteursInit();
  busInit();

  Serial.println();
  Serial.println("================================================");
  Serial.println("   EVITEMENT - 4 CAPTEURS - MODE AUTONOME");
  Serial.println("================================================");
  Serial.println();

  inventaireAfficher();
  distanceInit();
  inventaireAfficherDistance();

  // Premiere mesure de chaque capteur, pour verifier qu'ils repondent
  distanceAttendrePremiereMesure(500);
  Serial.println("--- Premiere mesure ---");
  for (uint8_t c = 0; c < 4; c++) {
    char buf[64];
    distanceTexte(c, buf, sizeof(buf), true);
    Serial.printf("  %s = %s\n", tof[c].nom, buf);
  }
  Serial.println("===========================================");
  Serial.println();

  if (!tof[CANAL_AVANT].present) {
    Serial.println("!! CAPTEUR AVANT ABSENT - DEMARRAGE ANNULE.");
    while (true) delay(1000);
  }

  if (!tof[CANAL_ARRIERE].present) {
    Serial.println("  Note : sans capteur arriere, le recul se fera");
    Serial.println("  a l'aveugle comme dans la version precedente.");
    Serial.println();
  }

  if (gyroInit()) {
    Serial.println("  MPU-6500                    OK (+/-250 deg/s)");
    delay(300);
    gyroCalibrer();
    Serial.println();
  } else {
    Serial.println("  MPU-6500 absent : virages moins precis.");
  }

  // --- Compte a rebours ---
  Serial.println("================================================");
  Serial.printf("  DEPART DANS %d SECONDES\n", COMPTE_A_REBOURS_S);
  Serial.println("  Poser le rover au sol et s'ecarter.");
  Serial.println("  ARRET : COUPER L'INTERRUPTEUR.");
  Serial.println("================================================");
  // Les capteurs restent lus pendant l'attente : au depart, leurs
  // mesures sont fraiches et aucun ne passe en ERR.
  for (int s = COMPTE_A_REBOURS_S; s > 0; s--) {
    Serial.printf("  %d\n", s);
    unsigned long debut = millis();
    while (millis() - debut < 1000) {
      distanceMettreAJour();
      gyroMettreAJour();
      delay(5);
    }
  }
  Serial.println();
  Serial.println(">>> DEPART");
  Serial.println();

  debugDemarrer();
  evitementDemarrer();
}

void loop() {
  debugEnvoyer();

  if (evitementEtat() == E_FIN) {
    delay(500);
    return;
  }

  gyroMettreAJour();
  distanceMettreAJour();
  moteursCalculerVitesses();
  evitementPas();
}
