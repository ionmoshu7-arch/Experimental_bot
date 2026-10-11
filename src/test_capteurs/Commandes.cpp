#include "Commandes.h"

#include "Gyro.h"
#include "Moteurs.h"
#include "Navigation.h"
#include "SuiviLigne.h"
#include "Telemetrie.h"

static unsigned long derniereCommandeMoteurs = 0;

// Une ligne en cours par source, pour ne pas melanger USB et WiFi.
// Lettres seules (c, z, s) : executees des la reception.
// Lignes "A v1 v2 v3 v4" : executees a la fin de ligne.
struct Lecteur {
  char ligne[48];
  uint8_t lg;
};
static Lecteur lecteurSerie, lecteurWifi;

void arreterMoteurs() {
  moteursStop();
  moteursActiver(false);
}

void arretGeneral() {
  suiviLigneArreter("[LIGNE] arret");
  navigationArreter("[NAV] arret");
  arreterMoteurs();
}

static bool modeAutonome() {
  return suiviLigneOccupe() || navigationActive();
}

static void executerLigne(Lecteur &l) {
  l.ligne[l.lg] = 0;
  char cmd = toupper(l.ligne[0]);
  const char* args = l.ligne + 1;
  int v[4];
  float kp, kd;

  if (cmd == 'A' && sscanf(args, "%d %d %d %d", &v[0], &v[1], &v[2], &v[3]) == 4) {
    // une commande manuelle reprend la main sur les modes autonomes
    if (suiviLigneOccupe()) suiviLigneArreter("[LIGNE] arret : commande manuelle");
    if (navigationActive()) navigationArreter("[NAV] arret : commande manuelle");
    derniereCommandeMoteurs = millis();
    for (int i = 0; i < 4; i++) moteurAppliquer(i, v[i]);
    moteursActiver(moteursActifs());
  } else if (cmd == 'L' && sscanf(args, "%d", &v[0]) == 1) {
    if (v[0]) { navigationArreter("[NAV] arret : suivi de ligne"); suiviLigneDemarrer(); }
    else suiviLigneArreter("[LIGNE] arret demande");
  } else if (cmd == 'N' && sscanf(args, "%d", &v[0]) == 1) {
    if (v[0]) { suiviLigneArreter("[LIGNE] arret : navigation"); navigationDemarrer(); }
    else navigationArreter("[NAV] arret demande");
  } else if (cmd == 'Q' && sscanf(args, "%d", &v[0]) == 1) {
    navigationVitesse(v[0]);
  } else if (cmd == 'K' && l.lg == 1) {
    suiviLigneCalibrer();
  } else if (cmd == 'R' && sscanf(args, "%d %d", &v[0], &v[1]) == 2) {
    suiviLigneRecherche(v[0] != 0, v[1]);
  } else if (cmd == 'P' && sscanf(args, "%f %f %d", &kp, &kd, &v[0]) == 3) {
    suiviLigneReglages(kp, kd, v[0]);
  } else if (l.lg) {
    journal("Commande inconnue : %s", l.ligne);
  }
  l.lg = 0;
}

static void traiterCaractere(Lecteur &l, char ch) {
  if (ch == '\n' || ch == '\r') { executerLigne(l); return; }
  if (l.lg == 0) {
    if (ch == 'c' || ch == 'C') {
      arretGeneral();     // la calibration bloque 1,5 s : rien ne doit tourner
      journal("[GYRO] calibration, ne pas bouger le rover");
      gyroCalibrer();
      journal("[GYRO] biais Z = %.4f deg/s", gyroBiaisZ());
      return;
    }
    if (ch == 'z' || ch == 'Z') { gyroRemettreCap(); journal("Cap remis a zero"); return; }
    if (ch == 's' || ch == 'S') { arretGeneral(); journal("[MOT] arret"); return; }
    if (ch == ' ') return;
  }
  if (l.lg < sizeof(l.ligne) - 1) l.ligne[l.lg++] = ch;
}

void commandesLire() {
  while (Serial.available()) traiterCaractere(lecteurSerie, Serial.read());
}

void commandesTraiterWifi(const char* texte, size_t n) {
  for (size_t i = 0; i < n; i++) traiterCaractere(lecteurWifi, texte[i]);
  // un message WebSocket est complet : une ligne sans fin est executee
  if (lecteurWifi.lg) executerLigne(lecteurWifi);
}

void commandesSurveiller() {
  if (modeAutonome()) return;   // suivi de ligne ou navigation pilotent les moteurs
  if (moteursActifs() &&
      millis() - derniereCommandeMoteurs > DELAI_SECURITE_MOTEURS_MS) {
    arreterMoteurs();
    journal("[MOT] arret de securite : plus de commande");
  }
}
