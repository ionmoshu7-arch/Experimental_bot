#include "SuiviLigne.h"

#include <Preferences.h>
#include "Config.h"
#include "Couleur.h"
#include "Distance.h"
#include "Moteurs.h"
#include "Telemetrie.h"

static const float SEUIL_LIGNE = 0.35;            // somme des noirceurs : en dessous, ligne perdue
static const float SEUIL_COTE = 0.15;             // |position| pour retenir le dernier cote vu
static const unsigned long DELAI_PERDUE_MS = 1500;
static const unsigned long PERIODE_REGULATION_MS = 25;   // ~ une mesure couleur
static const int DISTANCE_OBSTACLE_MM = 150;
static const int DISTANCE_REPRISE_MM = 220;
static const unsigned long DUREE_CALIBRATION_MS = 4000;
static const float CONTRASTE_MIN = 1.3;           // sol / ligne, sur le canal clair

static InfoLigne info = { L_ARRET, 0, { 0, 0, 0 }, false, 80, 4, 90 };
static uint16_t blanc[3], noir[3];                // canal clair C : sol et ligne
static uint16_t calMin[3], calMax[3];
static float positionPrec = 0;
static float dernierCote = 1;
static unsigned long dernierPas = 0, debutPerdue = 0, debutCalibration = 0;
static Preferences prefs;

void suiviLigneInit() {
  prefs.begin("ligne", false);
  info.kp = prefs.getFloat("kp", 80);
  info.kd = prefs.getFloat("kd", 4);
  info.vitesse = prefs.getInt("v", 90);
  info.calibre = prefs.isKey("blanc") && prefs.isKey("noir") &&
                 prefs.getBytes("blanc", blanc, sizeof(blanc)) == sizeof(blanc) &&
                 prefs.getBytes("noir", noir, sizeof(noir)) == sizeof(noir);
}

static bool capteursPrets() {
  for (auto &k : couleurs)
    if (!k.present || !k.valide) return false;
  return true;
}

// Noirceur de chaque capteur d'apres la calibration
static bool calculerNoirceur() {
  if (!info.calibre || !capteursPrets()) return false;
  for (int i = 0; i < 3; i++) {
    float n = (float) ((int) couleurs[i].c - noir[i]) / (blanc[i] - noir[i]);
    info.noirceur[i] = 1.0 - constrain(n, 0.0f, 1.0f);
  }
  return true;
}

void suiviLigneArreter(const char* raison) {
  if (info.etat == L_ARRET) return;
  if (info.etat != L_CALIBRATION) {
    moteursStop();
    moteursActiver(false);
  }
  info.etat = L_ARRET;
  if (raison) journal("%s", raison);
}

void suiviLigneDemarrer() {
  if (!capteursPrets()) { journal("[LIGNE] capteurs de couleur absents"); return; }
  if (!info.calibre)    { journal("[LIGNE] calibrer d'abord (bouton Calibrer)"); return; }
  if (info.etat == L_CALIBRATION) return;
  positionPrec = 0;
  dernierPas = millis();
  moteursActiver(true);
  info.etat = L_SUIT;
  journal("[LIGNE] suivi demarre (v=%d kp=%.0f kd=%.1f)", info.vitesse, info.kp, info.kd);
}

void suiviLigneCalibrer() {
  suiviLigneArreter(nullptr);
  if (!capteursPrets()) { journal("[LIGNE] capteurs de couleur absents"); return; }
  for (int i = 0; i < 3; i++) { calMin[i] = 0xFFFF; calMax[i] = 0; }
  debutCalibration = millis();
  info.etat = L_CALIBRATION;
  journal("[LIGNE] calibration 4 s : glisser le rover de part et d'autre de la ligne");
}

static void terminerCalibration() {
  info.etat = L_ARRET;
  for (int i = 0; i < 3; i++) {
    if (calMax[i] < calMin[i] * CONTRASTE_MIN || calMax[i] - calMin[i] < 20) {
      journal("[LIGNE] calibration ratee : capteur %s sans contraste (min %u, max %u). "
              "La LED des capteurs est-elle allumee ? Le capteur a-t-il vu la ligne ?",
              couleurs[i].nom, calMin[i], calMax[i]);
      return;
    }
  }
  for (int i = 0; i < 3; i++) { blanc[i] = calMax[i]; noir[i] = calMin[i]; }
  prefs.putBytes("blanc", blanc, sizeof(blanc));
  prefs.putBytes("noir", noir, sizeof(noir));
  info.calibre = true;
  journal("[LIGNE] calibration OK : sol %u/%u/%u, ligne %u/%u/%u",
          blanc[0], blanc[1], blanc[2], noir[0], noir[1], noir[2]);
}

void suiviLigneReglages(float kp, float kd, int vitesse) {
  info.kp = constrain(kp, 0.0f, 400.0f);
  info.kd = constrain(kd, 0.0f, 50.0f);
  info.vitesse = constrain(vitesse, 0, 255);
  prefs.putFloat("kp", info.kp);
  prefs.putFloat("kd", info.kd);
  prefs.putInt("v", info.vitesse);
  journal("[LIGNE] reglages : v=%d kp=%.0f kd=%.1f", info.vitesse, info.kp, info.kd);
}

void suiviLigneMettreAJour() {
  unsigned long maintenant = millis();

  if (info.etat == L_CALIBRATION) {
    for (int i = 0; i < 3; i++) {
      if (!couleurs[i].valide) continue;
      calMin[i] = min(calMin[i], couleurs[i].c);
      calMax[i] = max(calMax[i], couleurs[i].c);
    }
    if (maintenant - debutCalibration >= DUREE_CALIBRATION_MS) terminerCalibration();
    return;
  }

  bool mesure = calculerNoirceur();   // aussi a l'arret, pour l'affichage
  float somme = info.noirceur[0] + info.noirceur[1] + info.noirceur[2];
  if (mesure && somme >= SEUIL_LIGNE)
    info.position = (info.noirceur[2] - info.noirceur[0]) / somme;

  if (info.etat == L_ARRET) return;
  if (!mesure) { suiviLigneArreter("[LIGNE] capteurs de couleur indisponibles, arret"); return; }

  // Obstacle devant : pause, puis reprise quand la voie est libre
  int devant = distanceUtile(CANAL_AVANT);
  if (info.etat == L_OBSTACLE) {
    if (devant >= 0 && devant < DISTANCE_REPRISE_MM) return;
    info.etat = L_SUIT;
    dernierPas = maintenant;
    journal("[LIGNE] voie libre, reprise");
  } else if (devant >= 0 && devant < DISTANCE_OBSTACLE_MM) {
    moteursStop();
    info.etat = L_OBSTACLE;
    journal("[LIGNE] obstacle a %d mm, pause", devant);
    return;
  }

  if (maintenant - dernierPas < PERIODE_REGULATION_MS) return;
  float dt = (maintenant - dernierPas) / 1000.0;
  dernierPas = maintenant;

  // Ligne perdue : on pivote vers le dernier cote ou on l'a vue
  if (somme < SEUIL_LIGNE) {
    if (info.etat != L_PERDUE) { info.etat = L_PERDUE; debutPerdue = maintenant; }
    if (maintenant - debutPerdue > DELAI_PERDUE_MS) {
      suiviLigneArreter("[LIGNE] ligne perdue, arret");
      return;
    }
    int v = info.vitesse * 0.8;
    if (dernierCote >= 0) tournerDroite(v); else tournerGauche(v);
    return;
  }
  if (info.etat == L_PERDUE) {
    info.etat = L_SUIT;
    positionPrec = info.position;
  }

  // Regulateur PD : ligne a droite (position > 0) -> la droite ralentit
  float pos = info.position;
  if (fabs(pos) > SEUIL_COTE) dernierCote = pos;
  float derivee = (pos - positionPrec) / dt;
  positionPrec = pos;
  float correction = info.kp * pos + info.kd * derivee;
  int base = info.vitesse;
  moteursCotes(constrain((int) (base - correction), -255, 255),
               constrain((int) (base + correction), -255, 255));
}

bool suiviLigneOccupe() { return info.etat != L_ARRET; }
const InfoLigne& suiviLigneInfo() { return info; }

const char* suiviLigneNomEtat() {
  switch (info.etat) {
    case L_ARRET:       return "ARRET";
    case L_SUIT:        return "SUIT";
    case L_PERDUE:      return "PERDUE";
    case L_OBSTACLE:    return "OBSTACLE";
    case L_CALIBRATION: return "CALIBRATION";
  }
  return "?";
}
