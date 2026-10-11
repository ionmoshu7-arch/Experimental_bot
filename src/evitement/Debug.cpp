#include "Debug.h"

#include <Arduino.h>
#include "Config.h"
#include "Distance.h"
#include "Evitement.h"
#include "Gyro.h"
#include "Moteurs.h"
#include "Reglages.h"

static unsigned long dernierDebug = 0;
static unsigned long nbBoucles = 0;

void debugDemarrer() {
  dernierDebug = millis();
  nbBoucles = 0;
}

void debugEnvoyer() {
  nbBoucles++;
  if (DEBUG_PERIODE_MS == 0) return;
  unsigned long maintenant = millis();
  if (maintenant - dernierDebug < DEBUG_PERIODE_MS) return;
  unsigned long ecart = maintenant - dernierDebug;
  dernierDebug = maintenant;

  char av[64], ar[16], dr[16], ga[16];
  distanceTexte(CANAL_AVANT, av, sizeof(av), false);
  distanceTexte(CANAL_ARRIERE, ar, sizeof(ar), false);
  distanceTexte(CANAL_DROITE, dr, sizeof(dr), false);
  distanceTexte(CANAL_GAUCHE, ga, sizeof(ga), false);

  Serial.printf(
      "[DBG] t=%lu etat=%s av=%s ar=%s dr=%s ga=%s cap=%.1f cible=%.1f "
      "v_M1=%.0f v_M2=%.0f v_M3=%.0f v_M4=%.0f enc=%ld,%ld,%ld,%ld "
      "obst=%d annul=%d boucle_hz=%lu\n",
      maintenant / 1000, evitementNomEtat(), av, ar, dr, ga,
      gyroCap(), evitementCapCible(),
      moteurs[M1].vitesse, moteurs[M2].vitesse, moteurs[M3].vitesse, moteurs[M4].vitesse,
      moteurCompte(M1), moteurCompte(M2), moteurCompte(M3), moteurCompte(M4),
      evitementObstacles(), evitementReculsAnnules(),
      nbBoucles * 1000UL / ecart);
  nbBoucles = 0;
}
