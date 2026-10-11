#include "Navigation.h"

#include <math.h>
#include "Config.h"
#include "Distance.h"
#include "Gyro.h"
#include "Moteurs.h"
#include "Telemetrie.h"

// ---------- obstacles ----------
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
static const float RECUL_CM = 10.0;
static const int ARRIERE_MIN_MM = 280;         // place necessaire pour reculer
static const int ARRIERE_STOP_MM = 150;        // arret du recul
static const int V_RECUL = 90;

// ---------- collisions ----------
static const float SEUIL_CHOC_G = 0.6;         // variation brutale d'acceleration horizontale
static const int CONSIGNE_BLOCAGE_MIN = 60;    // moteurs vraiment commandes
static const float VITESSE_BLOCAGE_MM_S = 20;  // ... mais roues quasi immobiles
static const unsigned long DUREE_BLOCAGE_MS = 700;
static const unsigned long IGNORE_DEPART_MS = 400;   // a-coup au demarrage d'une manoeuvre

static const unsigned long PAUSE_MS = 300;
static const unsigned long MAX_VIRAGE_MS = 5000;
static const unsigned long MAX_RECUL_MS = 3000;

static InfoNav info = { N_ARRET, 0, 0, 0, 110, "" };
static unsigned long debutEtat = 0, dernierPas = 0, debutBlocage = 0;
static bool reculerEnsuite = false, grandVirage = false;
static float accelBase = 0;
static bool accelInitialise = false;

static void changer(EtatNav e) {
  info.etat = e;
  debutEtat = dernierPas = millis();
  debutBlocage = 0;
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
  moteursActiver(true);
  changer(N_AVANCE);
  journal("[NAV] navigation autonome demarree (vitesse %d)", info.vitesse);
}

void navigationVitesse(int vitesse) {
  info.vitesse = constrain(vitesse, 60, 200);
  journal("[NAV] vitesse %d", info.vitesse);
}

// Choc a l'accelerometre ou roues bloquees ; nullptr si rien
static const char* detecterCollision(unsigned long maintenant) {
  const AccelData &a = gyroAccel();
  float h = sqrtf(a.accelX * a.accelX + a.accelY * a.accelY);
  if (!accelInitialise) { accelBase = h; accelInitialise = true; }
  float ecart = fabsf(h - accelBase);
  accelBase += (h - accelBase) * 0.05;

  if (maintenant - debutEtat < IGNORE_DEPART_MS) return nullptr;
  if (ecart > SEUIL_CHOC_G) return "choc";

  bool commande = false;
  float vitesseMoy = 0;
  for (auto &m : moteurs) {
    if (abs(m.consigne) >= CONSIGNE_BLOCAGE_MIN) commande = true;
    vitesseMoy += fabsf(m.vitesse);
  }
  vitesseMoy /= 4;
  if (commande && vitesseMoy < VITESSE_BLOCAGE_MM_S) {
    if (!debutBlocage) debutBlocage = maintenant;
    else if (maintenant - debutBlocage > DUREE_BLOCAGE_MS) return "roues bloquees";
  } else {
    debutBlocage = 0;
  }
  return nullptr;
}

// Tourne vers le cote le plus degage (memes echelles : 25 cm max)
static void choisirVirage() {
  int g = distanceUtile(CANAL_GAUCHE), d = distanceUtile(CANAL_DROITE);
  int espaceG = g < 0 ? 0 : min(g, 250);
  int espaceD = d < 0 ? 0 : min(d, 250);
  bool aDroite = (abs(espaceG - espaceD) < 30) ? (random(2) == 1) : (espaceD > espaceG);
  float angle = grandVirage ? random(110, 161) : random(60, 121);
  info.capCible = gyroCap() + (aDroite ? -angle : angle);   // droite = negatif
  journal("[NAV] virage %s de %.0f deg (gauche %d mm, droite %d mm)",
          aDroite ? "a droite" : "a gauche", angle, g, d);
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
  changer(N_PAUSE);
}

static void collision(const char* cause) {
  info.collisions++;
  info.evenement = "COLLISION";
  journal("[NAV] COLLISION #%d detectee : %s", info.collisions, cause);
  moteursStop();
  reculerEnsuite = true;
  grandVirage = true;
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

void navigationMettreAJour() {
  if (info.etat == N_ARRET) return;
  unsigned long maintenant = millis();
  float dt = (maintenant - dernierPas) / 1000.0;
  dernierPas = maintenant;

  switch (info.etat) {

    case N_AVANCE: {
      int devant = distanceUtile(CANAL_AVANT);
      if (devant < 0) { navigationArreter("[NAV] capteur avant sans mesure, arret"); return; }
      if (devant < SEUIL_OBSTACLE_MM) { obstacle(devant); return; }
      const char* cause = detecterCollision(maintenant);
      if (cause) { collision(cause); return; }

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
      const char* cause = detecterCollision(maintenant);
      if (cause) {
        info.collisions++;
        info.evenement = "COLLISION";
        journal("[NAV] COLLISION #%d en reculant : %s", info.collisions, cause);
      }
      if (cause || (ar >= 0 && ar < ARRIERE_STOP_MM) ||
          parcouruCm >= RECUL_CM || maintenant - debutEtat > MAX_RECUL_MS) {
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
      const char* cause = detecterCollision(maintenant);
      if (cause || maintenant - debutEtat > MAX_VIRAGE_MS) {
        collision(cause ? cause : "virage impossible");
        return;
      }
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
