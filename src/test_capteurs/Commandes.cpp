#include "Commandes.h"

#include "Gyro.h"
#include "Moteurs.h"
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
  arreterMoteurs();
}

static void executerLigne(Lecteur &l) {
  l.ligne[l.lg] = 0;
  char cmd = toupper(l.ligne[0]);
  const char* args = l.ligne + 1;
  int v[4];
  float kp, kd;

  if (cmd == 'A' && sscanf(args, "%d %d %d %d", &v[0], &v[1], &v[2], &v[3]) == 4) {
    // une commande manuelle reprend la main sur le suivi de ligne
    if (suiviLigneOccupe()) suiviLigneArreter("[LIGNE] arret : commande manuelle");
    derniereCommandeMoteurs = millis();
    for (int i = 0; i < 4; i++) moteurAppliquer(i, v[i]);
    moteursActiver(moteursActifs());
  } else if (cmd == 'L' && sscanf(args, "%d", &v[0]) == 1) {
    if (v[0]) suiviLigneDemarrer();
    else suiviLigneArreter("[LIGNE] arret demande");
  } else if (cmd == 'K' && l.lg == 1) {
    suiviLigneCalibrer();
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
  if (suiviLigneOccupe()) return;   // le suivi de ligne pilote les moteurs
  if (moteursActifs() &&
      millis() - derniereCommandeMoteurs > DELAI_SECURITE_MOTEURS_MS) {
    arreterMoteurs();
    journal("[MOT] arret de securite : plus de commande");
  }
}
