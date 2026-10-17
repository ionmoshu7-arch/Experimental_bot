#include "Terrain.h"

#include <math.h>
#include "Config.h"
#include "Distance.h"
#include "Gyro.h"
#include "Moteurs.h"

static const float FILTRE_PENTE = 0.05;        // lissage de la pente
static const float FILTRE_VIBRATION = 0.05;
static const float SEUIL_BOSSE_G = 0.3;
static const unsigned long ECART_BOSSES_MS = 300;
static const float VITESSE_MIN_GLISSEMENT = 50;     // mm/s
static const int DISTANCE_MAX_GLISSEMENT = 2000;     // mm
static const unsigned long FENETRE_GLISSEMENT_MS = 250;

static InfoTerrain info = { 0, 0, 0, 0, -1 };
static float zeroTangage = 0, zeroRoulis = 0;
static float tangageBrut = 0, roulisBrut = 0;
static float azMoyen = 1, vibration2 = 0;
static bool initialise = false;
static unsigned long derniereBosse = 0;
static unsigned long debutFenetre = 0;
static int distanceDebut = -1;
static float parcoursEncodeurs = 0;
static unsigned long dernierPas = 0;

static float angleTangage(const AccelData &a) {
  return atan2f(a.accelX, sqrtf(a.accelY * a.accelY + a.accelZ * a.accelZ)) * RAD_TO_DEG;
}

static float angleRoulis(const AccelData &a) {
  return atan2f(a.accelY, a.accelZ) * RAD_TO_DEG;
}

void terrainCalibrer() {
  if (!gyroPresent()) return;
  const AccelData &a = gyroAccel();
  // pas encore de mesure (accelerometre a 0) : on calibrera a la premiere
  if (sqrtf(a.accelX * a.accelX + a.accelY * a.accelY + a.accelZ * a.accelZ) < 0.5f) {
    initialise = false;
    return;
  }
  zeroTangage = tangageBrut = angleTangage(a);
  zeroRoulis = roulisBrut = angleRoulis(a);
  azMoyen = a.accelZ;
  vibration2 = 0;
  derniereBosse = millis();      // pas de bosse comptee juste apres
  initialise = true;
}

void terrainRemettreAZero() { info.bosses = 0; }

static void mesurerGlissement(unsigned long maintenant) {
  float v = 0;
  for (auto &m : moteurs) v += m.vitesse;
  v /= 4;                                          // mm/s, > 0 en avant
  bool tourne = fabsf(gyroVitesseZ()) > 15;
  int devant = tof[CANAL_AVANT].etat == M_OK ? tof[CANAL_AVANT].brut : -1;

  bool mesurable = v > VITESSE_MIN_GLISSEMENT && !tourne &&
                   devant > 0 && devant < DISTANCE_MAX_GLISSEMENT;
  if (!mesurable) {
    distanceDebut = -1;
    if (fabsf(v) < 10) info.glissement = -1;      // a l'arret : inconnu
    return;
  }
  if (distanceDebut < 0) {
    distanceDebut = devant;
    parcoursEncodeurs = 0;
    debutFenetre = dernierPas = maintenant;
    return;
  }
  parcoursEncodeurs += v * (maintenant - dernierPas) / 1000.0f;
  dernierPas = maintenant;
  if (maintenant - debutFenetre < FENETRE_GLISSEMENT_MS) return;

  // rapport entre ce que mesure le capteur avant et ce que disent les roues
  float reel = distanceDebut - devant;
  if (parcoursEncodeurs > 10) {
    float g = constrain(1.0f - reel / parcoursEncodeurs, 0.0f, 1.0f);
    info.glissement = info.glissement < 0 ? g : info.glissement * 0.7f + g * 0.3f;
  }
  distanceDebut = devant;
  parcoursEncodeurs = 0;
  debutFenetre = maintenant;
}

void terrainMettreAJour() {
  if (!gyroPresent()) return;
  const AccelData &a = gyroAccel();
  if (!initialise) terrainCalibrer();

  tangageBrut += (angleTangage(a) - tangageBrut) * FILTRE_PENTE;
  roulisBrut += (angleRoulis(a) - roulisBrut) * FILTRE_PENTE;
  info.tangage = tangageBrut - zeroTangage;
  info.roulis = roulisBrut - zeroRoulis;

  // vibrations verticales autour de la moyenne
  azMoyen += (a.accelZ - azMoyen) * 0.02f;
  float hp = a.accelZ - azMoyen;
  vibration2 += (hp * hp - vibration2) * FILTRE_VIBRATION;
  info.rugosite = sqrtf(vibration2);

  unsigned long maintenant = millis();
  if (fabsf(hp) > SEUIL_BOSSE_G && maintenant - derniereBosse > ECART_BOSSES_MS) {
    info.bosses++;
    derniereBosse = maintenant;
  }

  mesurerGlissement(maintenant);
}

const InfoTerrain& terrainInfo() { return info; }
