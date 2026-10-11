#include "Commandes.h"

#include <Arduino.h>
#include "Gyro.h"
#include "Moteurs.h"

static unsigned long derniereCommandeMoteurs = 0;

// Lettres seules (c, z, s) : executees des la frappe.
// Lignes "A v1 v2 v3 v4" : executees a la fin de ligne.
static char ligne[48];
static uint8_t lg = 0;

void arreterMoteurs() {
  moteursStop();
  moteursActiver(false);
}

static void executerLigne() {
  ligne[lg] = 0;
  int v[4];
  if ((ligne[0] == 'A' || ligne[0] == 'a') &&
      sscanf(ligne + 1, "%d %d %d %d", &v[0], &v[1], &v[2], &v[3]) == 4) {
    derniereCommandeMoteurs = millis();
    for (int i = 0; i < 4; i++) moteurAppliquer(i, v[i]);
    moteursActiver(moteursActifs());
  } else if (lg) {
    Serial.printf("Commande inconnue : %s\n", ligne);
  }
  lg = 0;
}

void commandesLire() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { executerLigne(); continue; }
    if (lg == 0) {
      if (ch == 'c' || ch == 'C') {
        arreterMoteurs();   // la calibration bloque 1,5 s : rien ne doit tourner
        gyroCalibrer();
        continue;
      }
      if (ch == 'z' || ch == 'Z') { gyroRemettreCap(); Serial.println("Cap remis a zero"); continue; }
      if (ch == 's' || ch == 'S') { arreterMoteurs(); Serial.println("[MOT] arret"); continue; }
      if (ch == ' ') continue;
    }
    if (lg < sizeof(ligne) - 1) ligne[lg++] = ch;
  }
}

void commandesSurveiller() {
  if (moteursActifs() &&
      millis() - derniereCommandeMoteurs > DELAI_SECURITE_MOTEURS_MS) {
    arreterMoteurs();
    Serial.println("[MOT] arret de securite : plus de commande");
  }
}
