/*
 * 15_TEST_CAPTEURS
 * ------------------------------------------------------------
 * LECTURE DE TOUS LES CAPTEURS + TEST DES MOTEURS
 *
 *   - 4 capteurs de distance (TCA9548A canaux 0 a 3)
 *   - 3 capteurs de couleur TCS34725 (canaux 4, 5, 6)
 *   - Gyroscope / accelerometre MPU-6500 (0x68)
 *   - 4 moteurs + encodeurs, pilotables un par un
 *
 * LES MOTEURS NE TOURNENT QUE SUR COMMANDE, et s'arretent seuls si
 * aucune commande n'arrive pendant 400 ms. Tester ROUES EN L'AIR.
 *
 * Port serie a 921600 bauds. Page de suivi : web/rover_moniteur.html
 *
 * Fichiers :
 *   main.cpp        demarrage et boucle
 *   Telemetrie.cpp  bloc texte + trames [DATA] JSON
 *   Commandes.cpp   commandes serie (c, z, s, A ...) et securite moteurs
 *   lib/Rover/      materiel commun (moteurs, capteurs, gyro, bus)
 * ------------------------------------------------------------
 */

#include <Arduino.h>
#include "Bus.h"
#include "Couleur.h"
#include "Distance.h"
#include "Gyro.h"
#include "Inventaire.h"
#include "Moteurs.h"

#include "Commandes.h"
#include "Telemetrie.h"

// 921600 bauds : une trame JSON part en ~10 ms au lieu de ~80 ms.
// Le moniteur de VS Code et la page web doivent utiliser la meme vitesse.
const unsigned long VITESSE_SERIE = 921600;

void setup() {
  // Tampon d'envoi de 4 Ko : Serial.print rend la main tout de suite
  Serial.setTxBufferSize(4096);
  Serial.begin(VITESSE_SERIE);
  delay(1000);

  moteursInit();   // moteurs arretes, ils ne tournent que sur commande
  busInit();

  Serial.println();
  Serial.println("================================================");
  Serial.println("   TEST CAPTEURS + MOTEURS");
  Serial.println("   commandes : c = calibrer, z = cap a 0, s = stop moteurs");
  Serial.println("================================================");

  bool mux = muxPresent();
  Serial.printf("  TCA9548A 0x73          %s\n", mux ? "OK" : "ABSENT");
  if (mux) {
    distanceInit();
    couleurInit();
    muxFermer();
  }
  inventaireAfficherDistance();
  inventaireAfficherCouleur();

  if (gyroInit()) {
    Serial.println("  MPU-6500 0x68          OK (+/-250 deg/s)");
    delay(300);
    gyroCalibrer();
  } else {
    Serial.println("  MPU-6500 0x68          ABSENT");
  }
  Serial.println("================================================");

  telemetrieDemarrer();
}

void loop() {
  commandesLire();

  // Aucune lecture n'attend : chaque capteur n'est lu que s'il a une
  // nouvelle mesure, et le gyroscope est integre a chaque tour.
  gyroMettreAJour();
  distanceMettreAJour();
  couleurMettreAJour();
  moteursCalculerVitesses();
  commandesSurveiller();

  telemetrieMettreAJour();
}
