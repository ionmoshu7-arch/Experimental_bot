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
 * PILOTAGE ET SUIVI :
 *   - par USB : port serie a 921600 bauds
 *   - par WiFi : http://rover.local/ (page servie par le rover)
 * La page est web/rover_moniteur.html, embarquee dans le firmware.
 * Identifiants WiFi : include/secrets.h (non versionne).
 *
 * Fichiers :
 *   main.cpp        demarrage et boucle
 *   Telemetrie.cpp  bloc texte + trames [DATA] JSON
 *   Commandes.cpp   commandes (c, z, s, A, L, K, P) et securite moteurs
 *   SuiviLigne.cpp  suivi de ligne noire (3 capteurs de couleur, PD)
 *   Navigation.cpp  navigation autonome : cap au gyroscope, obstacles,
 *                   detection de collision
 *   Reseau.cpp      WiFi, page web, WebSocket, mises a jour OTA
 *   RemoteXYControle.cpp  pilotage par l'application RemoteXY (port 6377)
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
#include "Odometrie.h"
#include "Terrain.h"

#include "Commandes.h"
#include "Navigation.h"
#include "RemoteXYControle.h"
#include "Reseau.h"
#include "SuiviLigne.h"
#include "Telemetrie.h"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copier include/secrets.example.h en include/secrets.h et y mettre le WiFi"
#endif

// web/rover_moniteur.html, embarque par board_build.embed_txtfiles
extern const char PAGE_CONTROLE[] asm("_binary_web_rover_moniteur_html_start");

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
  Serial.println("   version du " __DATE__ " " __TIME__);
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
  suiviLigneInit();
  Serial.printf("  Suivi de ligne         %s\n",
                suiviLigneInfo().calibre ? "calibre" : "a calibrer (bouton Calibrer de la page)");

  if (gyroInit()) {
    Serial.println("  MPU-6500 0x68          OK (+/-250 deg/s)");
    delay(300);
    gyroCalibrer();
    terrainCalibrer();                    // sol plat du depart = pente nulle
  } else {
    Serial.println("  MPU-6500 0x68          ABSENT");
  }
  // --- WiFi : pilotage et suivi depuis la page web ---
  reseauSurMessage(commandesTraiterWifi);
  reseauSurDeconnexion(arretGeneral);     // page fermee ou reseau coupe
  reseauSurOta(arretGeneral);             // rien ne tourne pendant une mise a jour
  reseauDemarrer(WIFI_SSID, WIFI_MOT_DE_PASSE, OTA_MOT_DE_PASSE, PAGE_CONTROLE);
  remoteXYDemarrer();                     // application telephone, meme WiFi
  Serial.println("================================================");

  odometrieInit();                        // origine de la carte : ici
  telemetrieDemarrer();
}

void loop() {
  reseauMettreAJour();
  remoteXYMettreAJour();
  commandesLire();

  // Aucune lecture n'attend : chaque capteur n'est lu que s'il a une
  // nouvelle mesure, et le gyroscope est integre a chaque tour.
  gyroMettreAJour();
  distanceMettreAJour();
  couleurMettreAJour();
  moteursCalculerVitesses();
  odometrieMettreAJour();
  terrainMettreAJour();
  suiviLigneMettreAJour();
  navigationMettreAJour();
  commandesSurveiller();

  telemetrieMettreAJour();
}
