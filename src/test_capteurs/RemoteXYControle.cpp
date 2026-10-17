#include "RemoteXYControle.h"

#include <Arduino.h>

#if __has_include("RemoteXYInterface.h")

#include <WiFi.h>
#include <RemoteXY.h>
#include "RemoteXYInterface.h"   // genere par remotexy.com : RemoteXY_CONF_PROGMEM + struct RemoteXY

#include "Commandes.h"
#include "Config.h"
#include "Distance.h"
#include "Gyro.h"
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
static const unsigned long PERIODE_AFFICHAGE_MS = 200;

static bool joystickActif = false;
static uint8_t ligneAvant = 0, navAvant = 0, stopAvant = 0, connecteAvant = 0;
static unsigned long dernierAffichage = 0;

void remoteXYDemarrer() {
  RemoteXYGui* gui = RemoteXYEngine.addGui(RemoteXY_CONF_PROGMEM, &RemoteXY);
  gui->addConnectionServer(new ReseauExistant(), PORT_REMOTEXY);
  RemoteXY.slider_vitesse = 60;   // valeur de depart du curseur, en %
  Serial.printf("  RemoteXY               port %u (application Android/iOS)\n", PORT_REMOTEXY);
}

static void afficherEtat() {
  const char* mode = suiviLigneOccupe() ? "Ligne" : navigationActive() ? "Navigation"
                   : joystickActif ? "Manuel" : "Arret";
  const char* detail = suiviLigneOccupe() ? suiviLigneNomEtat()
                     : navigationActive() ? navigationNomEtat() : "";
  snprintf(RemoteXY.text_etat, sizeof(RemoteXY.text_etat), "%s %s  col:%d",
           mode, detail, navigationInfo().collisions);

  char av[16], ar[16], ga[16], dr[16];
  distanceTexte(CANAL_AVANT, av, sizeof(av), false);
  distanceTexte(CANAL_ARRIERE, ar, sizeof(ar), false);
  distanceTexte(CANAL_GAUCHE, ga, sizeof(ga), false);
  distanceTexte(CANAL_DROITE, dr, sizeof(dr), false);
  snprintf(RemoteXY.text_capteurs, sizeof(RemoteXY.text_capteurs),
           "Av%s Ar%s G%s D%s cap%.0f", av, ar, ga, dr, gyroCap());
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
      journal("[RXY] application RemoteXY deconnectee, arret");
    }
  }
  if (!RemoteXY.connect_flag) return;

  // --- STOP ---
  if (RemoteXY.button_stop && !stopAvant) {
    arretGeneral();
    journal("[RXY] STOP");
  }
  stopAvant = RemoteXY.button_stop;

  // --- interrupteurs des modes autonomes (sur changement) ---
  if (RemoteXY.switch_ligne != ligneAvant) {
    ligneAvant = RemoteXY.switch_ligne;
    if (ligneAvant) {
      navigationArreter("[NAV] arret : suivi de ligne");
      suiviLigneDemarrer();
    } else {
      suiviLigneArreter("[LIGNE] arret depuis RemoteXY");
    }
  }
  if (RemoteXY.switch_nav != navAvant) {
    navAvant = RemoteXY.switch_nav;
    if (navAvant) {
      suiviLigneArreter("[LIGNE] arret : navigation");
      navigationDemarrer();
    } else {
      navigationArreter("[NAV] arret depuis RemoteXY");
    }
  }
  // un mode arrete tout seul (ligne perdue, STOP de la page...) :
  // l'interrupteur du telephone revient a OFF
  if (RemoteXY.switch_ligne && !suiviLigneOccupe()) RemoteXY.switch_ligne = ligneAvant = 0;
  if (RemoteXY.switch_nav && !navigationActive())   RemoteXY.switch_nav = navAvant = 0;

  // --- joystick : pilotage manuel ---
  int x = RemoteXY.joystick_x, y = RemoteXY.joystick_y;   // -100..100
  if (abs(x) > ZONE_MORTE || abs(y) > ZONE_MORTE) {
    int vmax = constrain((int) RemoteXY.slider_vitesse, 0, 100) * 255 / 100;
    float gauche = y + x, droite = y - x;                  // x > 0 : vers la droite
    float m = max(100.0f, max(fabsf(gauche), fabsf(droite)));
    commandeManuelle((int) (droite * vmax / m), (int) (gauche * vmax / m));
    joystickActif = true;
    RemoteXY.switch_ligne = ligneAvant = 0;                // le joystick prend la main
    RemoteXY.switch_nav = navAvant = 0;
  } else if (joystickActif) {
    arreterMoteurs();                                      // joystick relache
    joystickActif = false;
  }

  // --- retour d'informations vers le telephone ---
  if (millis() - dernierAffichage >= PERIODE_AFFICHAGE_MS) {
    dernierAffichage = millis();
    afficherEtat();
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
