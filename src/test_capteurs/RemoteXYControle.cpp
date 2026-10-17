#include "RemoteXYControle.h"

#include <Arduino.h>

#if __has_include("RemoteXYInterface.h")

#include <WiFi.h>
#include <RemoteXY.h>
#include "RemoteXYInterface.h"   // genere par remotexy.com : RemoteXY_CONF_PROGMEM + struct RemoteXY

#include "Commandes.h"
#include "Moteurs.h"
#include "Navigation.h"
#include "SuiviLigne.h"
#include "Telemetrie.h"

// RemoteXY sur la connexion WiFi deja etablie par Reseau.cpp. La classe
// CRemoteXYNet_WiFi de la bibliotheque refait WiFi.begin() et couperait
// la page web : on ne lui laisse que le serveur TCP.
class ReseauExistant : public CRemoteXYNet {
 public:
  uint8_t configured() override { return WiFi.status() == WL_CONNECTED; }
  CRemoteXYServer* createServer(uint16_t port) override { return new CRemoteXYServer_WiFi(this, port); }
  uint8_t hasInternet() override { return 1; }
  CRemoteXYClient* newClient() override { return new CRemoteXYClient_WiFi(); }
};

static const int ZONE_MORTE = 10;                  // joystick -100..100

// Positions du selecteur de mode (A, B, C dans l'application)
enum ModeTelephone : uint8_t { MODE_MANUEL = 0, MODE_LIGNE = 1, MODE_NAV = 2 };

static bool joystickActif = false;
static uint8_t modeAvant = MODE_MANUEL, stopAvant = 0, connecteAvant = 0;

void remoteXYDemarrer() {
  RemoteXYGui* gui = RemoteXYEngine.addGui(RemoteXY_CONF_PROGMEM, &RemoteXY);
  gui->addConnectionServer(new ReseauExistant(), PORT_REMOTEXY);
  RemoteXY.slider_vitesse = 60;   // valeur de depart du curseur, en %
  RemoteXY.selecteur_mode = MODE_MANUEL;
  Serial.printf("  RemoteXY               port %u (application Android/iOS)\n", PORT_REMOTEXY);
}

// Le selecteur change de position : on lance le mode choisi
static void changerMode(uint8_t mode) {
  switch (mode) {
    case MODE_LIGNE:
      navigationArreter("[NAV] arret : suivi de ligne");
      suiviLigneDemarrer();
      break;
    case MODE_NAV:
      suiviLigneArreter("[LIGNE] arret : navigation");
      navigationDemarrer();
      break;
    default:   // manuel : les modes autonomes s'arretent
      suiviLigneArreter("[LIGNE] arret : mode manuel (RemoteXY)");
      navigationArreter("[NAV] arret : mode manuel (RemoteXY)");
      break;
  }
}

// Le selecteur du telephone revient sur A (manuel)
static void selecteurManuel() {
  RemoteXY.selecteur_mode = modeAvant = MODE_MANUEL;
}

void remoteXYMettreAJour() {
  RemoteXYEngine.handler();

  // --- connexion de l'application ---
  if (RemoteXY.connect_flag != connecteAvant) {
    connecteAvant = RemoteXY.connect_flag;
    if (connecteAvant) {
      journal("[RXY] application RemoteXY connectee");
    } else {
      arretGeneral();            // telephone parti : rien ne doit rouler
      joystickActif = false;
      selecteurManuel();
      journal("[RXY] application RemoteXY deconnectee, arret");
    }
  }
  if (!RemoteXY.connect_flag) return;

  // --- STOP ---
  if (RemoteXY.button_stop && !stopAvant) {
    arretGeneral();
    selecteurManuel();
    journal("[RXY] STOP");
  }
  stopAvant = RemoteXY.button_stop;

  // --- selecteur de mode (sur changement) ---
  if (RemoteXY.selecteur_mode != modeAvant) {
    modeAvant = RemoteXY.selecteur_mode;
    changerMode(modeAvant);
  }
  // un mode arrete tout seul (ligne perdue, STOP de la page, pas
  // calibre...) : le selecteur revient sur A
  if ((modeAvant == MODE_LIGNE && !suiviLigneOccupe()) ||
      (modeAvant == MODE_NAV && !navigationActive())) {
    selecteurManuel();
  }

  // --- joystick : pilotage manuel ---
  int x = RemoteXY.joystick_x, y = RemoteXY.joystick_y;   // -100..100
  if (abs(x) > ZONE_MORTE || abs(y) > ZONE_MORTE) {
    int vmax = constrain((int) RemoteXY.slider_vitesse, 0, 100) * 255 / 100;
    float gauche = y + x, droite = y - x;                  // x > 0 : vers la droite
    float m = max(100.0f, max(fabsf(gauche), fabsf(droite)));
    commandeManuelle((int) (droite * vmax / m), (int) (gauche * vmax / m));
    joystickActif = true;
    selecteurManuel();                                     // le joystick prend la main
  } else if (joystickActif) {
    arreterMoteurs();                                      // joystick relache
    joystickActif = false;
  }
}

bool remoteXYDisponible() { return true; }
bool remoteXYConnecte() { return RemoteXY.connect_flag; }

#else   // pas d'interface RemoteXY : fonctions vides

void remoteXYDemarrer() {
  Serial.println("  RemoteXY               desactive (pas de RemoteXYInterface.h, voir docs/RemoteXY.md)");
}
void remoteXYMettreAJour() {}
bool remoteXYDisponible() { return false; }
bool remoteXYConnecte() { return false; }

#endif
