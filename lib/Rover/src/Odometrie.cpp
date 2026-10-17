#include "Odometrie.h"

#include <math.h>
#include "Config.h"
#include "Gyro.h"
#include "Moteurs.h"

static Pose pose = { 0, 0, 0, 0 };
static long comptesPrec[4] = { 0, 0, 0, 0 };
static float capPrec = 0;      // cap du gyroscope au tour precedent
static float capOrigine = 0;   // cap du gyroscope a l'origine

static void lireComptes(long c[4]) {
  for (int i = 0; i < 4; i++) c[i] = moteurCompteTotal(i);
}

void odometrieInit() {
  lireComptes(comptesPrec);
  capOrigine = capPrec = gyroCap();
}

void odometrieRemettreAZero() {
  pose = { 0, 0, 0, 0 };
  odometrieInit();
}

void odometrieMettreAJour() {
  long c[4];
  lireComptes(c);
  long somme = 0;
  for (int i = 0; i < 4; i++) {
    somme += c[i] - comptesPrec[i];
    comptesPrec[i] = c[i];
  }
  float d = somme / 4.0f * MM_PAR_IMPULSION;      // positif = vers l'avant

  float cap = gyroCap();
  // cap moyen pendant le deplacement, en radians
  float milieu = ((capPrec + cap) / 2 - capOrigine) * (float) DEG_TO_RAD;
  capPrec = cap;

  pose.x += d * cosf(milieu);
  pose.y += d * sinf(milieu);
  pose.cap = cap - capOrigine;
  pose.distance += fabsf(d);
}

void odometrieRecalerCap() {
  capPrec = gyroCap();
  capOrigine = capPrec - pose.cap;
}

const Pose& odometriePose() { return pose; }
