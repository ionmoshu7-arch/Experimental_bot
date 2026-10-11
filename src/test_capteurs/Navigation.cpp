#include "Navigation.h"

#include <math.h>
#include "Config.h"
#include "Distance.h"
#include "Gyro.h"
#include "Moteurs.h"
#include "Telemetrie.h"

// ---------- obstacles vus par les capteurs ----------
static const int SEUIL_OBSTACLE_MM = 280;      // arret devant un obstacle
static const int SEUIL_TRES_PRES_MM = 160;     // en dessous : recul avant de tourner
static const int DEBUT_RALENTI_MM = 600;       // ralentit a partir de cette distance
static const float FACTEUR_VITESSE_MIN = 0.55;
static const int MUR_GAUCHE_MM = 150;          // s'ecarte d'un mur plus proche
static const int MUR_DROITE_MM = 120;          // (portee du capteur droit : 25 cm)
static const float ECART_MUR_DEG_S = 25;

// ---------- cap et virages (gyroscope) ----------
static const float KP_CAP = 4.0;               // correction par degre d'ecart
static const float KD_CAP = 0.4;               // amortissement par deg/s
static const float KP_VIRAGE = 2.5;
static const int V_VIRAGE_MIN = 75, V_VIRAGE_MAX = 140;
static const float TOLERANCE_VIRAGE_DEG = 3.0;

// ---------- recul ----------
static const float RECUL_CM = 12.0;
static const float GRAND_RECUL_CM = 20.0;
static const int ARRIERE_MIN_MM = 280;         // place necessaire pour reculer
static const int ARRIERE_STOP_MM = 150;        // arret du recul
static const int V_RECUL = 90;

// ---------- collisions invisibles aux capteurs (pied de chaise...) ----------
// 1. choc : variation brutale de l'acceleration horizontale
static const float SEUIL_CHOC_G = 0.35;
// 2. blocage : une roue (ou un cote) tourne beaucoup moins vite que
//    d'habitude pour la meme consigne. "D'habitude" est appris en roulant.
static const float RATIO_BLOCAGE = 0.35;       // < 35 % du rendement normal
static const unsigned long DUREE_BLOCAGE_MS = 300;
static const int CONSIGNE_MIN = 50;            // en dessous, rendement non mesure
// 3. deviation : le rover est pousse hors de son cap en ligne droite
static const float DEVIATION_MAX_DEG = 20;
// 4. virage bloque : moteurs en rotation mais le gyroscope ne tourne pas
static const float ROTATION_MIN_DEG_S = 8;
static const unsigned long DUREE_VIRAGE_BLOQUE_MS = 700;

static const unsigned long IGNORE_DEPART_MS = 500;   // a-coup au debut d'une manoeuvre
static const unsigned long PAUSE_MS = 300;
static const unsigned long MAX_VIRAGE_MS = 5000;
static const unsigned long MAX_RECUL_MS = 3000;
// collisions repetees : on insiste, puis on abandonne
static const int COLLISIONS_INSISTER = 3;      // en 15 s : grand recul + demi-tour
static const int COLLISIONS_ABANDON = 6;       // en 30 s : arret, rover coince

static InfoNav info = { N_ARRET, 0, 0, 0, 110, "", 0 };
static unsigned long debutEtat = 0, dernierPas = 0;
static unsigned long debutBlocageD = 0, debutBlocageG = 0, debutVirageBloque = 0;
static bool reculerEnsuite = false, grandVirage = false;
static int coteImpact = 0;                     // +1 obstacle a droite, -1 a gauche, 0 inconnu
static float distanceRecul = RECUL_CM;
static float accelBaseX = 0, accelBaseY = 0;
static bool accelInitialise = false;
static float rendementRef = 0;                 // (mm/s) par unite de consigne, appris
static int nbEchantillons = 0;
static unsigned long instantsCollision[COLLISIONS_ABANDON];
static int idxCollision = 0;

static void changer(EtatNav e) {
  info.etat = e;
  debutEtat = dernierPas = millis();
  debutBlocageD = debutBlocageG = debutVirageBloque = 0;
}

void navigationArreter(const char* raison) {
  if (info.etat == N_ARRET) return;
  moteursStop();
  moteursActiver(false);
  info.etat = N_ARRET;
  if (raison) journal("%s", raison);
}

void navigationDemarrer() {
  if (!gyroPresent())               { journal("[NAV] gyroscope absent : navigation impossible"); return; }
  if (!tof[CANAL_AVANT].present)    { journal("[NAV] capteur avant absent : navigation impossible"); return; }
  if (info.etat != N_ARRET) return;
  info.capCible = gyroCap();
  info.evenement = "depart";
  accelInitialise = false;
  for (auto &t : instantsCollision) t = 0;
  moteursActiver(true);
  changer(N_AVANCE);
  journal("[NAV] navigation autonome demarree (vitesse %d)", info.vitesse);
}

void navigationVitesse(int vitesse) {
  info.vitesse = constrain(vitesse, 60, 200);
  journal("[NAV] vitesse %d", info.vitesse);
}

// ---------- mesures ----------

// Variation d'acceleration horizontale depuis la moyenne recente (g)
static float mesurerChoc() {
  const AccelData &a = gyroAccel();
  if (!accelInitialise) {
    accelBaseX = a.accelX; accelBaseY = a.accelY;
    accelInitialise = true;
  }
  float dx = a.accelX - accelBaseX, dy = a.accelY - accelBaseY;
  accelBaseX += dx * 0.05;
  accelBaseY += dy * 0.05;
  return sqrtf(dx * dx + dy * dy);
}

// Vitesse mesuree et consigne d'un cote (droite : M1+M3, gauche : M2+M4)
static void cote(bool droite, float &vitesse, float &consigne) {
  const Moteur &a = moteurs[droite ? M1 : M2];
  const Moteur &b = moteurs[droite ? M3 : M4];
  vitesse = (a.vitesse + b.vitesse) / 2;
  consigne = (a.consigne + b.consigne) / 2.0;
}

// Rendement = vitesse / consigne, ou -1 si la consigne est trop faible
static float rendement(bool droite) {
  float v, c;
  cote(droite, v, c);
  if (fabsf(c) < CONSIGNE_MIN) return -1;
  return v / c;
}

// Apprend le rendement normal pendant qu'il roule librement
static void apprendreRendement(float rD, float rG) {
  if (rD <= 0 || rG <= 0) return;
  float r = (rD + rG) / 2;
  if (nbEchantillons < 40) {
    rendementRef += (r - rendementRef) / (nbEchantillons + 1);
    nbEchantillons++;
  } else if (r > rendementRef * 0.6 && r < rendementRef * 1.6) {
    rendementRef += (r - rendementRef) * 0.02;
  }
  info.rendement = nbEchantillons >= 40 ? rendementRef : 0;
}

static bool blocageCote(float r, unsigned long &debut, unsigned long maintenant) {
  if (r >= 0 && r < rendementRef * RATIO_BLOCAGE) {
    if (!debut) debut = maintenant;
    return maintenant - debut > DUREE_BLOCAGE_MS;
  }
  debut = 0;
  return false;
}

// Cote de l'obstacle d'apres la rotation subie : si l'avant droit
// touche, la droite freine, la gauche pousse et le rover pivote a
// droite (cap qui diminue, vitesse Z negative).
static int coteDepuisRotation() {
  float gz = gyroVitesseZ();
  if (gz < -10) return +1;
  if (gz > 10) return -1;
  return 0;
}

// ---------- reactions ----------

static void choisirVirage() {
  bool aDroite;
  if (coteImpact != 0) {
    aDroite = coteImpact < 0;          // on s'eloigne du cote touche
  } else {
    int g = distanceUtile(CANAL_GAUCHE), d = distanceUtile(CANAL_DROITE);
    int espaceG = g < 0 ? 0 : min(g, 250);   // meme echelle : portee droite 25 cm
    int espaceD = d < 0 ? 0 : min(d, 250);
    aDroite = (abs(espaceG - espaceD) < 30) ? (random(2) == 1) : (espaceD > espaceG);
  }
  float angle = grandVirage ? random(140, 181) : random(70, 111);
  info.capCible = gyroCap() + (aDroite ? -angle : angle);   // droite = negatif
  journal("[NAV] virage %s de %.0f deg", aDroite ? "a droite" : "a gauche", angle);
  coteImpact = 0;
  changer(N_TOURNE);
}

static void commencerRecul() {
  int ar = distanceUtile(CANAL_ARRIERE);
  if (tof[CANAL_ARRIERE].present && ar < ARRIERE_MIN_MM) {
    journal("[NAV] pas de place derriere (%d mm) : virage sur place", ar);
    choisirVirage();
    return;
  }
  info.capCible = gyroCap();
  moteursRemettreAZero();
  changer(N_RECULE);
}

static void obstacle(int devant) {
  info.obstacles++;
  info.evenement = "obstacle";
  journal("[NAV] obstacle #%d a %d mm", info.obstacles, devant);
  moteursStop();
  reculerEnsuite = devant < SEUIL_TRES_PRES_MM;
  grandVirage = false;
  coteImpact = 0;
  distanceRecul = RECUL_CM;
  changer(N_PAUSE);
}

static void collision(const char* cause, int cote) {
  unsigned long maintenant = millis();
  info.collisions++;
  info.evenement = "COLLISION";
  instantsCollision[idxCollision] = maintenant;
  idxCollision = (idxCollision + 1) % COLLISIONS_ABANDON;

  int recentes15 = 0, recentes30 = 0;
  for (auto t : instantsCollision) {
    if (!t) continue;
    if (maintenant - t < 15000) recentes15++;
    if (maintenant - t < 30000) recentes30++;
  }
  journal("[NAV] COLLISION #%d : %s%s", info.collisions, cause,
          cote > 0 ? " (obstacle a droite)" : cote < 0 ? " (obstacle a gauche)" : "");
  moteursStop();

  if (recentes30 >= COLLISIONS_ABANDON) {
    navigationArreter("[NAV] rover coince : trop de collisions, arret");
    return;
  }
  bool insister = recentes15 >= COLLISIONS_INSISTER;
  if (insister) journal("[NAV] collisions repetees : grand recul et demi-tour");
  reculerEnsuite = true;
  grandVirage = insister;
  distanceRecul = insister ? GRAND_RECUL_CM : RECUL_CM;
  coteImpact = insister ? 0 : cote;
  changer(N_PAUSE);
}

// Roule en tenant le cap : base > 0 en avant, < 0 en arriere
static void roulerAuCap(float base) {
  float ecart = gyroCap() - info.capCible;    // > 0 : le rover a derive a gauche
  float correction = KP_CAP * ecart + KD_CAP * gyroVitesseZ();
  float limite = fabsf(base) * 0.8;
  correction = constrain(correction, -limite, limite);
  moteursCotes((int) (base - correction), (int) (base + correction));
}

// ---------- boucle ----------

void navigationMettreAJour() {
  if (info.etat == N_ARRET) return;
  unsigned long maintenant = millis();
  float dt = (maintenant - dernierPas) / 1000.0;
  dernierPas = maintenant;
  float choc = mesurerChoc();   // toujours, pour suivre la moyenne
  bool demarrage = maintenant - debutEtat < IGNORE_DEPART_MS;

  switch (info.etat) {

    case N_AVANCE: {
      int devant = distanceUtile(CANAL_AVANT);
      if (devant < 0) { navigationArreter("[NAV] capteur avant sans mesure, arret"); return; }
      if (devant < SEUIL_OBSTACLE_MM) { obstacle(devant); return; }

      if (!demarrage) {
        if (choc > SEUIL_CHOC_G) { collision("choc", coteDepuisRotation()); return; }

        float ecart = gyroCap() - info.capCible;
        if (fabsf(ecart) > DEVIATION_MAX_DEG) {
          collision("pousse hors de son cap", ecart < 0 ? +1 : -1);
          return;
        }

        float rD = rendement(true), rG = rendement(false);
        if (nbEchantillons >= 40) {
          bool bD = blocageCote(rD, debutBlocageD, maintenant);
          bool bG = blocageCote(rG, debutBlocageG, maintenant);
          if (bD && bG) { collision("roues bloquees", 0); return; }
          if (bD)       { collision("cote droit freine", +1); return; }
          if (bG)       { collision("cote gauche freine", -1); return; }
        }
        if (!debutBlocageD && !debutBlocageG) apprendreRendement(rD, rG);
      }

      // mur trop proche sur un cote : le cap cible s'en ecarte
      int g = distanceUtile(CANAL_GAUCHE), d = distanceUtile(CANAL_DROITE);
      if (g > 0 && g < MUR_GAUCHE_MM) info.capCible -= ECART_MUR_DEG_S * dt;
      if (d > 0 && d < MUR_DROITE_MM) info.capCible += ECART_MUR_DEG_S * dt;

      // ralentit en approchant d'un obstacle
      float f = (float) (devant - SEUIL_OBSTACLE_MM) / (DEBUT_RALENTI_MM - SEUIL_OBSTACLE_MM);
      f = FACTEUR_VITESSE_MIN + (1 - FACTEUR_VITESSE_MIN) * constrain(f, 0.0f, 1.0f);
      roulerAuCap(info.vitesse * f);
      break;
    }

    case N_PAUSE:
      moteursStop();
      if (maintenant - debutEtat >= PAUSE_MS) {
        if (reculerEnsuite) commencerRecul();
        else choisirVirage();
      }
      break;

    case N_RECULE: {
      int ar = distanceUtile(CANAL_ARRIERE);
      float parcouruCm = compteMoyen() / IMPULSIONS_PAR_TOUR * CIRCONFERENCE_CM;
      bool bloque = false;
      if (!demarrage && nbEchantillons >= 40) {
        bool bD = blocageCote(rendement(true), debutBlocageD, maintenant);
        bool bG = blocageCote(rendement(false), debutBlocageG, maintenant);
        bloque = (bD && bG) || choc > SEUIL_CHOC_G;
      }
      if (bloque) {
        info.collisions++;
        info.evenement = "COLLISION";
        journal("[NAV] COLLISION #%d en reculant", info.collisions);
      }
      if (bloque || (ar >= 0 && ar < ARRIERE_STOP_MM) ||
          parcouruCm >= distanceRecul || maintenant - debutEtat > MAX_RECUL_MS) {
        moteursStop();
        choisirVirage();
        return;
      }
      roulerAuCap(-V_RECUL);
      break;
    }

    case N_TOURNE: {
      float ecart = info.capCible - gyroCap();   // > 0 : il faut tourner a gauche
      if (fabsf(ecart) < TOLERANCE_VIRAGE_DEG) {
        moteursStop();
        info.evenement = "en route";
        changer(N_AVANCE);   // garde exactement le cap vise
        return;
      }
      // moteurs en rotation mais le rover ne pivote pas : il est coince
      if (!demarrage && fabsf(gyroVitesseZ()) < ROTATION_MIN_DEG_S) {
        if (!debutVirageBloque) debutVirageBloque = maintenant;
        else if (maintenant - debutVirageBloque > DUREE_VIRAGE_BLOQUE_MS) {
          collision("virage bloque", ecart > 0 ? -1 : +1);
          return;
        }
      } else {
        debutVirageBloque = 0;
      }
      if (maintenant - debutEtat > MAX_VIRAGE_MS) { collision("virage trop long", 0); return; }
      int v = constrain((int) (KP_VIRAGE * fabsf(ecart)), V_VIRAGE_MIN, V_VIRAGE_MAX);
      if (ecart > 0) tournerGauche(v); else tournerDroite(v);
      break;
    }

    default:
      break;
  }
}

bool navigationActive() { return info.etat != N_ARRET; }
const InfoNav& navigationInfo() { return info; }

const char* navigationNomEtat() {
  switch (info.etat) {
    case N_ARRET:  return "ARRET";
    case N_AVANCE: return "AVANCE";
    case N_PAUSE:  return "PAUSE";
    case N_RECULE: return "RECULE";
    case N_TOURNE: return "TOURNE";
  }
  return "?";
}
