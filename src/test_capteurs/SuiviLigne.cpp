#include "SuiviLigne.h"

#include <Preferences.h>
#include "Config.h"
#include "Couleur.h"
#include "Distance.h"
#include "Gyro.h"
#include "Moteurs.h"
#include "Telemetrie.h"

static const float SEUIL_LIGNE = 0.35;            // somme des noirceurs : en dessous, ligne perdue
static const float SEUIL_COTE = 0.15;             // |position| pour retenir le dernier cote vu
static const unsigned long PERIODE_REGULATION_MS = 25;   // ~ une mesure couleur
static const int DISTANCE_OBSTACLE_MM = 150;
static const int DISTANCE_REPRISE_MM = 220;
static const unsigned long DUREE_CALIBRATION_MS = 4000;
static const float CONTRASTE_MIN = 1.3;           // sol / ligne, sur le canal clair

// ---------- recherche de la ligne perdue ----------
static const unsigned long GRACE_MS = 150;        // tout droit : petites coupures
static const unsigned long MAX_RECHERCHE_MS = 12000;
static const unsigned long MAX_ETAPE_MS = 4000;
static const float TOLERANCE_CAP_DEG = 4;
static const int ARRIERE_STOP_MM = 120;
static const float MS_PAR_DEGRE_SANS_GYRO = 12;   // estimation si gyroscope absent

static InfoLigne info = { L_ARRET, 0, { 0, 0, 0 }, false, 80, 4, 90, true, R_TOUTES, 0, 0 };
static uint16_t blanc[3], noir[3];                // canal clair C : sol et ligne
static uint16_t calMin[3], calMax[3];
static float positionPrec = 0;
static float dernierCote = 1;
static unsigned long dernierPas = 0, debutPerdue = 0, debutCalibration = 0;
static Preferences prefs;

enum TypeEtape : uint8_t { ET_RECUL, ET_CAP };
struct Etape { TypeEtape type; float valeur; };   // RECUL : cm ; CAP : degres depuis la perte
static Etape etapes[8];
static float capPerte = 0, offsetSansGyro = 0;
static unsigned long debutRecherche = 0, debutEtape = 0;
static bool etapeCommencee = false;

void suiviLigneInit() {
  prefs.begin("ligne", false);
  info.kp = prefs.getFloat("kp", 80);
  info.kd = prefs.getFloat("kd", 4);
  info.vitesse = prefs.getInt("v", 90);
  info.recherche = prefs.getBool("rech", true);
  info.strategie = prefs.getUChar("strat", R_TOUTES);
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
  if (!couleurActifs())  { journal("[LIGNE] capteurs de couleur eteints : les rallumer d'abord"); return; }
  if (!capteursPrets()) { journal("[LIGNE] capteurs de couleur absents"); return; }
  if (!info.calibre)    { journal("[LIGNE] calibrer d'abord (bouton Calibrer)"); return; }
  if (info.etat == L_CALIBRATION) return;
  positionPrec = 0;
  dernierPas = millis();
  moteursActiver(true);
  info.etat = L_SUIT;
  journal("[LIGNE] suivi demarre (v=%d kp=%.0f kd=%.1f, recherche %s)", info.vitesse,
          info.kp, info.kd, info.recherche ? suiviLigneNomStrategie(info.strategie) : "desactivee");
}

void suiviLigneCalibrer() {
  suiviLigneArreter(nullptr);
  if (!couleurActifs())  { journal("[LIGNE] capteurs de couleur eteints : les rallumer d'abord"); return; }
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

void suiviLigneRecherche(bool active, int strategie) {
  info.recherche = active;
  info.strategie = constrain(strategie, (int) R_PIVOT, (int) R_TOUTES);
  prefs.putBool("rech", info.recherche);
  prefs.putUChar("strat", info.strategie);
  journal("[LIGNE] recherche de ligne %s (%s)", active ? "activee" : "desactivee",
          suiviLigneNomStrategie(info.strategie));
}

// ---------- recherche de la ligne ----------

static void ajouterEtape(TypeEtape type, float valeur) {
  if (info.nbEtapes < 8) etapes[info.nbEtapes++] = { type, valeur };
}

static void demarrerRecherche() {
  // ligne vue a droite en dernier -> chercher d'abord a droite (cap negatif)
  float s = dernierCote >= 0 ? -1 : 1;
  info.nbEtapes = 0;
  switch (info.strategie) {
    case R_PIVOT:
      ajouterEtape(ET_CAP, s * 120);
      break;
    case R_RECUL:
      ajouterEtape(ET_RECUL, 10);
      ajouterEtape(ET_CAP, s * 90);
      ajouterEtape(ET_CAP, -s * 90);
      break;
    case R_BALAYAGE:
      ajouterEtape(ET_CAP, s * 35);
      ajouterEtape(ET_CAP, -s * 35);
      ajouterEtape(ET_CAP, s * 80);
      ajouterEtape(ET_CAP, -s * 80);
      ajouterEtape(ET_CAP, s * 170);
      break;
    default:   // R_TOUTES
      ajouterEtape(ET_CAP, s * 60);
      ajouterEtape(ET_CAP, 0);
      ajouterEtape(ET_RECUL, 8);
      ajouterEtape(ET_CAP, -s * 80);
      ajouterEtape(ET_CAP, s * 80);
      ajouterEtape(ET_CAP, s * 170);
      break;
  }
  info.etape = 0;
  capPerte = gyroCap();
  offsetSansGyro = 0;
  debutRecherche = debutEtape = millis();
  etapeCommencee = false;
  info.etat = L_RECHERCHE;
  journal("[LIGNE] ligne perdue : recherche %s", suiviLigneNomStrategie(info.strategie));
}

static void executerRecherche(unsigned long maintenant) {
  if (maintenant - debutRecherche > MAX_RECHERCHE_MS || info.etape >= info.nbEtapes) {
    suiviLigneArreter("[LIGNE] ligne introuvable apres recherche, arret");
    return;
  }
  const Etape &e = etapes[info.etape];
  int v = constrain((int) (info.vitesse * 0.8), 70, 140);
  bool fini = false;

  if (!etapeCommencee) {
    etapeCommencee = true;
    debutEtape = maintenant;
    if (e.type == ET_RECUL) moteursRemettreAZero();
  }

  if (e.type == ET_RECUL) {
    int ar = distanceUtile(CANAL_ARRIERE);
    float cm = compteMoyen() / IMPULSIONS_PAR_TOUR * CIRCONFERENCE_CM;
    if (cm >= e.valeur || (ar >= 0 && ar < ARRIERE_STOP_MM) || maintenant - debutEtape > 3000)
      fini = true;
    else
      reculer(v);
  } else if (gyroPresent()) {
    float ecart = (capPerte + e.valeur) - gyroCap();   // > 0 : tourner a gauche
    if (fabsf(ecart) < TOLERANCE_CAP_DEG || maintenant - debutEtape > MAX_ETAPE_MS) {
      fini = true;
    } else {
      int vt = constrain((int) (2.5 * fabsf(ecart)), 70, v);
      if (ecart > 0) tournerGauche(vt); else tournerDroite(vt);
    }
  } else {
    // sans gyroscope : duree estimee du pivot
    float delta = e.valeur - offsetSansGyro;
    if (maintenant - debutEtape > fabsf(delta) * MS_PAR_DEGRE_SANS_GYRO) {
      fini = true;
      offsetSansGyro = e.valeur;
    } else if (delta > 0) {
      tournerGauche(v);
    } else {
      tournerDroite(v);
    }
  }

  if (fini) {
    moteursStop();
    info.etape++;
    etapeCommencee = false;
  }
}

// ---------- boucle ----------

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

  // ---- ligne absente ----
  if (somme < SEUIL_LIGNE) {
    if (info.etat == L_SUIT) {
      info.etat = L_PERDUE;
      debutPerdue = maintenant;
    }
    if (info.etat == L_PERDUE) {
      if (maintenant - debutPerdue < GRACE_MS) {
        int v = info.vitesse * 0.7;
        moteursCotes(v, v);
        return;
      }
      if (!info.recherche) {
        suiviLigneArreter("[LIGNE] ligne perdue, arret (recherche desactivee)");
        return;
      }
      demarrerRecherche();
    }
    executerRecherche(maintenant);
    return;
  }

  // ---- ligne visible ----
  if (info.etat == L_RECHERCHE) journal("[LIGNE] ligne retrouvee");
  if (info.etat == L_PERDUE || info.etat == L_RECHERCHE) {
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
    case L_RECHERCHE:   return "RECHERCHE";
    case L_OBSTACLE:    return "OBSTACLE";
    case L_CALIBRATION: return "CALIBRATION";
  }
  return "?";
}

const char* suiviLigneNomStrategie(uint8_t strategie) {
  switch (strategie) {
    case R_PIVOT:    return "PIVOT";
    case R_RECUL:    return "RECUL";
    case R_BALAYAGE: return "BALAYAGE";
    default:         return "TOUTES";
  }
}
