/*
 * CAMERA DU ROVER - Diymore ESP32-S3-WROOM CAM (N16R8, OV2640, carte TF)
 * ------------------------------------------------------------
 * Carte separee du rover, sur le meme WiFi :
 *   - video en direct     http://camera.local:81/stream
 *   - photos, reglages    http://camera.local/ (voir ServeurCamera.h)
 *   - synchronisation     WebSocket du rover (LiaisonRover.h)
 * La page du rover (http://rover.local/) affiche la video et pilote
 * la camera ; la page de la camera reste disponible pour les tests.
 *
 * Alimentation definitive : broche 5V de la carte, depuis le rail 5,6 V
 * du rover a travers une diode 1N4007 (~4,9 V). Pas le rail 3,3 V des
 * capteurs : les pointes de courant du WiFi le perturberaient.
 *
 * Fichiers :
 *   main.cpp           demarrage, WiFi, mises a jour OTA
 *   Camera.cpp         capteur OV2640 (resolution, qualite, images/s)
 *   ServeurCamera.cpp  serveurs HTTP : video, photo, reglages, etat
 *   Stockage.cpp       carte TF : photos
 *   Enregistrement.cpp video AVI a la demande sur la carte TF
 *   Journal.cpp        journal de navigation du rover (fichiers tournants)
 *   LiaisonRover.cpp   synchronisation avec le rover
 *   Pont.cpp           console des 2 cartes sur le cable USB (460800 bauds)
 *   RelaisMaj.cpp      programmer le rover a travers la camera
 * Identifiants WiFi : include/secrets.h (non versionne), comme le rover.
 * ------------------------------------------------------------
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

#include "Camera.h"
#include "Enregistrement.h"
#include "Journal.h"
#include "LiaisonRover.h"
#include "Pont.h"
#include "RelaisMaj.h"
#include "ServeurCamera.h"
#include "Stockage.h"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copier include/secrets.example.h en include/secrets.h et y mettre le WiFi"
#endif

static const char* NOM_HOTE = "camera";      // camera.local
static bool otaDemarre = false;
static bool etaitConnecte = false;

static void demarrerOta() {
  ArduinoOTA.setHostname(NOM_HOTE);           // demarre aussi mDNS
  ArduinoOTA.setPassword(OTA_MOT_DE_PASSE);
  ArduinoOTA.onStart([]() { Serial.println("[OTA] mise a jour en cours..."); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("[OTA] erreur %u\n", e); });
  ArduinoOTA.begin();
  MDNS.addService("http", "tcp", PORT_HTTP);
  otaDemarre = true;
}

void setup() {
  Serial.setRxBufferSize(8192);               // blocs du programme du rover
  Serial.begin(VITESSE_SERIE_CAMERA);
  delay(500);
  Serial.println();
  Serial.println("================================================");
  Serial.println("   CAMERA DU ROVER - ESP32-S3 N16R8 + OV2640");
  Serial.println("   version du " __DATE__ " " __TIME__);
  Serial.println("================================================");
  Serial.printf("  PSRAM : %u Ko   flash : %u Mo\n",
                (unsigned) (ESP.getPsramSize() / 1024), (unsigned) (ESP.getFlashChipSize() >> 20));

  bool cam = cameraInit();
  cameraChargerRotation();
  Serial.printf("  Camera : %s\n", cam ? cameraModele() : "ERREUR (nappe bien enfoncee ?)");
  stockageInit();
  enregistrementInit();
  journalInit();

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(NOM_HOTE);
  WiFi.setSleep(false);                       // video plus fluide
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_MOT_DE_PASSE);
  Serial.printf("[WIFI] connexion a \"%s\"", WIFI_SSID);
  unsigned long debut = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - debut < 15000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  serveurCameraDemarrer(OTA_MOT_DE_PASSE);
  liaisonRoverDemarrer();
  relaisMajInit(OTA_MOT_DE_PASSE);
  pontDemarrer();
}

void loop() {
  bool connecte = WiFi.status() == WL_CONNECTED;
  if (connecte && !etaitConnecte) {
    Serial.printf("[WIFI] connecte : http://%s.local/  ou  http://%s/  (%d dBm)\n",
                  NOM_HOTE, WiFi.localIP().toString().c_str(), WiFi.RSSI());
    Serial.printf("[WIFI] video : http://%s:%u/stream\n",
                  WiFi.localIP().toString().c_str(), PORT_FLUX);
    WiFi.setSleep(false);                     // economie d'energie WiFi coupee (latence)
    if (!otaDemarre) demarrerOta();
  } else if (!connecte && etaitConnecte) {
    Serial.println("[WIFI] connexion perdue, reconnexion automatique...");
  }
  etaitConnecte = connecte;

  if (otaDemarre) ArduinoOTA.handle();
  liaisonRoverMettreAJour();
  pontMettreAJour();
  journalMettreAJour();

  static unsigned long dernierLog = 0;
  if (millis() - dernierLog >= 5000) {
    dernierLog = millis();
    Serial.printf("[CAM] %s %s q%d  %.1f img/s  %d ecran(s)  rover %s\n",
                  cameraModele(), cameraNomResolution(), cameraQualiteActuelle(),
                  cameraImagesParSeconde(), serveurNbFlux(),
                  liaisonRoverConnectee() ? "connecte" : "non connecte");
  }
  delay(2);
}
