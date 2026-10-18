#include "Navigation.h"

#include <math.h>
#include "Config.h"
#include "Distance.h"
#include "Gyro.h"
#include "Moteurs.h"
#include "Odometrie.h"
#include <Preferences.h>
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
// collisions repetees : on insiste, puis degagement
static const int COLLISIONS_INSISTER = 3;      // en 15 s : grand recul + demi-tour
static const int COLLISIONS_ABANDON = 6;       // en 30 s : rover coince -> degagement

// ---------- degagement ----------
static const int SECTEURS = 24;                // tour complet par pas de 15 deg
static const int V_SCAN = 85;
static const unsigned long MAX_SCAN_MS = 12000;
static const int SORTIE_MIN_MM = 350;          // distance libre pour une sortie
static const int V_SECOUSSE = 200;             // plus de puissance pour se decoincer
static const unsigned long PHASE_SECOUSSE_MS = 350;
static const uint8_t SECOUSSE[] = { 0, 1, 0, 2, 0, 1, 0, 2 };   // 0 recule, 1 gauche, 2 droite
static const int NB_PHASES_SECOUSSE = sizeof(SECOUSSE);
static const int MAX_ESSAIS_DEGAGEMENT = 4;
static const unsigned long ROUTE_LIBRE_MS = 3000;   // sans collision : degage

// ---------- modes but / mur / orienter ----------
static const float RAYON_ARRIVEE_MM = 150;
static const float ANGLE_TOURNER_D_ABORD = 60;   // ecart de cap au-dela : virage sur place
static const int MUR_CIBLE_GAUCHE_MM = 150;       // distance au mur longe
static const int MUR_CIBLE_DROITE_MM = 120;       // (portee du capteur droit : 25 cm)
static const float KP_MUR_DEG_S_PAR_MM = 0.3;
static const float CHERCHE_MUR_DEG_S = 20;        // plus de mur : on tourne vers lui

static InfoNav info = { N_ARRET, 0, 0, 0, 110, "", 0, 0, MAX_ESSAIS_DEGAGEMENT,
                        true, true, true, true, false, 0, 0, 0 };
static bool arretApresVirage = false;   // mode orienter
static bool butAtteint = false;
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

static int scanDistance[SECTEURS];             // meilleure distance vue par secteur, -1 = rien
static float scanDepart = 0;
static int sensScan = 1;                       // +1 tourne a gauche, -1 a droite
static bool scanInverse = false;
static int phaseSecousse = 0;
static unsigned long debutPhase = 0;

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
  arretApresVirage = false;
  if (raison) journal("%s", raison);
}

void navigationDemarrer() {
  if (!gyroPresent())               { journal("[NAV] gyroscope absent : navigation impossible"); return; }
  if (!tof[CANAL_AVANT].present)    { journal("[NAV] capteur avant absent : navigation impossible"); return; }
  if (info.etat != N_ARRET) return;
  info.capCible = gyroCap();
  info.evenement = "depart";
  info.essaiDegagement = 0;
  accelInitialise = false;
  for (auto &t : instantsCollision) t = 0;
  moteursActiver(true);
  changer(N_AVANCE);
  journal("[NAV] navigation autonome demarree (vitesse %d)", info.vitesse);
}

// ---------- options ----------
void navigationOptionsCharger() {
  Preferences p;
  p.begin("nav", true);
  info.optCollision = p.getBool("collision", true);
  info.optDegagement = p.getBool("degagement", true);
  info.optCap = p.getBool("cap", true);
  info.optMurs = p.getBool("murs", true);
  p.end();
}

bool navigationOption(const char* nom, bool active) {
  bool* o = !strcmp(nom, "collision") ? &info.optCollision
          : !strcmp(nom, "degagement") ? &info.optDegagement
          : !strcmp(nom, "cap") ? &info.optCap
          : !strcmp(nom, "murs") ? &info.optMurs : nullptr;
  if (!o) return false;
  *o = active;
  Preferences p;
  p.begin("nav", false);
  p.putBool(nom, active);
  p.end();
  journal("[NAV] option %s : %s", nom, active ? "active" : "desactivee");
  return true;
}

// ---------- modes ----------
static float ecartAngle(float a) {               // ramene dans -180..180
  return fmodf(a + 540.0f, 360.0f) - 180.0f;
}

void navigationLibre() {
  info.aBut = false;
  info.mur = 0;
  arretApresVirage = false;
}

void navigationAllerVers(float x, float y) {
  navigationLibre();
  info.butX = x; info.butY = y;
  info.aBut = true;
  butAtteint = false;
  journal("[NAV] but (%.2f, %.2f) m", x / 1000, y / 1000);
  if (info.etat == N_ARRET) navigationDemarrer();
}

void navigationLongerMur(int8_t cote) {
  navigationLibre();
  info.mur = cote >= 0 ? 1 : -1;
  journal("[NAV] longe le mur %s", info.mur > 0 ? "gauche" : "droit");
  if (info.etat == N_ARRET) navigationDemarrer();
}

void navigationOrienter(float capCarte) {
  navigationLibre();
  butAtteint = false;
  if (info.etat == N_ARRET) navigationDemarrer();
  if (info.etat == N_ARRET) return;              // demarrage refuse
  info.capCible = gyroCap() + ecartAngle(capCarte - odometriePose().cap);
  arretApresVirage = true;
  changer(N_TOURNE);
}

bool navigationButAtteint() {
  bool b = butAtteint;
  butAtteint = false;
  return b;
}

const char* navigationNomMode() {
  if (arretApresVirage) return "orienter";
  if (info.aBut) return "but";
  if (info.mur > 0) return "mur gauche";
  if (info.mur < 0) return "mur droit";
  return "libre";
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

// ---------- degagement : le rover est coince ----------

static void commencerScan() {
  if (info.essaiDegagement >= MAX_ESSAIS_DEGAGEMENT) {
    navigationArreter("[NAV] impossible de se degager apres 4 essais : aidez le rover");
    return;
  }
  info.essaiDegagement++;
  for (auto &d : scanDistance) d = -1;
  scanDepart = gyroCap();
  sensScan = random(2) ? 1 : -1;
  scanInverse = false;
  info.evenement = "DEGAGEMENT";
  journal("[NAV] degagement, essai %d/%d : tour sur place pour trouver une sortie",
          info.essaiDegagement, MAX_ESSAIS_DEGAGEMENT);
  changer(N_DEGAGE_SCAN);
}

static void commencerSecousse(const char* raison) {
  journal("[NAV] degagement : %s, secousses pour se decoincer", raison);
  phaseSecousse = 0;
  debutPhase = millis();
  changer(N_DEGAGE_SECOUSSE);
}

static void commencerDegagement() {
  if (!info.optDegagement) {
    navigationArreter("[NAV] rover coince, degagement automatique desactive : arret");
    return;
  }
  for (auto &t : instantsCollision) t = 0;   // nouvel historique pour la suite
  info.essaiDegagement = 0;
  journal("[NAV] rover coince : degagement");
  commencerScan();
}

// Fin du tour : part vers le secteur le plus degage
static void choisirSortie() {
  int meilleur = -1;
  for (int s = 0; s < SECTEURS; s++)
    if (scanDistance[s] > (meilleur < 0 ? -1 : scanDistance[meilleur])) meilleur = s;
  if (meilleur < 0 || scanDistance[meilleur] < SORTIE_MIN_MM) {
    commencerSecousse(meilleur < 0 ? "aucune mesure" : "aucune sortie assez large");
    return;
  }
  float angle = (meilleur + 0.5) * (360.0 / SECTEURS);
  // apres un tour complet : virage le plus court vers la sortie (-180..180)
  float delta = fmodf(scanDepart + sensScan * angle - gyroCap() + 540.0f, 360.0f) - 180.0f;
  info.capCible = gyroCap() + delta;
  journal("[NAV] sortie trouvee a %.0f deg %s (%d mm libres)", angle > 180 ? 360 - angle : angle,
          (sensScan > 0) == (angle <= 180) ? "a gauche" : "a droite", scanDistance[meilleur]);
  changer(N_TOURNE);
}

static void obstacle(int devant) {
  info.obstacles++;
  info.evenement = "obstacle";
  journal("[NAV] obstacle #%d a %d mm", info.obstacles, devant);
  moteursStop();
  reculerEnsuite = devant < SEUIL_TRES_PRES_MM;
  grandVirage = false;
  coteImpact = -info.mur;        // longe un mur : tourne du cote oppose (0 sinon)
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

  // deja en degagement : la sortie choisie est bouchee, essai suivant
  if (info.essaiDegagement > 0) {
    commencerSecousse("nouvelle collision");
    return;
  }
  if (recentes30 >= COLLISIONS_ABANDON) {
    commencerDegagement();
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
  if (!info.optCap && !info.aBut && info.mur == 0) {   // option coupee : meme consigne partout
    moteursCotes((int) base, (int) base);
    return;
  }
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

      // degagement reussi : 3 s de route sans collision
      if (info.essaiDegagement > 0 && maintenant - debutEtat > ROUTE_LIBRE_MS) {
        info.essaiDegagement = 0;
        info.evenement = "degage";
        journal("[NAV] rover degage, navigation normale");
      }

      if (!demarrage && info.optCollision) {
        if (choc > SEUIL_CHOC_G) { collision("choc", coteDepuisRotation()); return; }

        float ecart = gyroCap() - info.capCible;
        bool capTenu = info.optCap || info.aBut || info.mur != 0;
        if (capTenu && fabsf(ecart) > DEVIATION_MAX_DEG) {
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

      // vers le but : cap cible oriente vers le point, virage d'abord si trop d'ecart
      if (info.aBut) {
        const Pose &po = odometriePose();
        float dx = info.butX - po.x, dy = info.butY - po.y;
        if (sqrtf(dx * dx + dy * dy) < RAYON_ARRIVEE_MM) {
          info.aBut = false;
          butAtteint = true;
          navigationArreter("[NAV] but atteint");
          return;
        }
        float ecartBut = ecartAngle(atan2f(dy, dx) * RAD_TO_DEG - po.cap);
        info.capCible = gyroCap() + ecartBut;
        if (fabsf(ecartBut) > ANGLE_TOURNER_D_ABORD) { changer(N_TOURNE); return; }
      }

      // longer un mur : regulateur sur la distance laterale
      if (info.mur != 0) {
        uint8_t canal = info.mur > 0 ? CANAL_GAUCHE : CANAL_DROITE;
        int cible = info.mur > 0 ? MUR_CIBLE_GAUCHE_MM : MUR_CIBLE_DROITE_MM;
        int dm = tof[canal].etat == M_OK ? tof[canal].brut : -1;
        float vitesseCap = dm > 0
          ? constrain((dm - cible) * KP_MUR_DEG_S_PAR_MM, -30.0f, 30.0f)   // trop loin : vers le mur
          : CHERCHE_MUR_DEG_S;                                             // plus de mur : le chercher
        info.capCible += info.mur * vitesseCap * dt;                       // mur gauche = cap +
      }

      // mur trop proche sur un cote : le cap cible s'en ecarte (option)
      if (info.optMurs) {
        int g = distanceUtile(CANAL_GAUCHE), d = distanceUtile(CANAL_DROITE);
        if (info.mur <= 0 && g > 0 && g < MUR_GAUCHE_MM) info.capCible -= ECART_MUR_DEG_S * dt;
        if (info.mur >= 0 && d > 0 && d < MUR_DROITE_MM) info.capCible += ECART_MUR_DEG_S * dt;
      }

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
      if (!demarrage && info.optCollision && nbEchantillons >= 40) {
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
        if (arretApresVirage) {          // mode orienter : c'est fini
          arretApresVirage = false;
          butAtteint = true;
          navigationArreter("[NAV] orientation atteinte");
          return;
        }
        info.evenement = "en route";
        changer(N_AVANCE);   // garde exactement le cap vise
        return;
      }
      // moteurs en rotation mais le rover ne pivote pas : il est coince
      if (!demarrage && info.optCollision && fabsf(gyroVitesseZ()) < ROTATION_MIN_DEG_S) {
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

    case N_DEGAGE_SCAN: {
      float tourne = (gyroCap() - scanDepart) * sensScan;   // degres parcourus
      int devant = distanceUtile(CANAL_AVANT);
      if (tourne >= 0 && tourne < 360 && devant >= 0) {
        int s = (int) (tourne / (360.0 / SECTEURS)) % SECTEURS;
        scanDistance[s] = max(scanDistance[s], devant);
      }
      if (tourne >= 360 || maintenant - debutEtat > MAX_SCAN_MS) {
        moteursStop();
        choisirSortie();
        return;
      }
      // ne pivote pas : essaie dans l'autre sens, puis secousses
      if (!demarrage && fabsf(gyroVitesseZ()) < ROTATION_MIN_DEG_S) {
        if (!debutVirageBloque) debutVirageBloque = maintenant;
        else if (maintenant - debutVirageBloque > DUREE_VIRAGE_BLOQUE_MS) {
          moteursStop();
          if (!scanInverse) {
            journal("[NAV] degagement : pivot bloque, essai dans l'autre sens");
            scanInverse = true;
            sensScan = -sensScan;
            scanDepart = gyroCap();
            for (auto &d : scanDistance) d = -1;
            changer(N_DEGAGE_SCAN);
          } else {
            commencerSecousse("pivot bloque des deux cotes");
          }
          return;
        }
      } else {
        debutVirageBloque = 0;
      }
      if (sensScan > 0) tournerGauche(V_SCAN); else tournerDroite(V_SCAN);
      break;
    }

    case N_DEGAGE_SECOUSSE: {
      if (maintenant - debutPhase >= PHASE_SECOUSSE_MS) {
        phaseSecousse++;
        debutPhase = maintenant;
      }
      if (phaseSecousse >= NB_PHASES_SECOUSSE) {
        moteursStop();
        commencerScan();
        return;
      }
      int ar = distanceUtile(CANAL_ARRIERE);
      switch (SECOUSSE[phaseSecousse]) {
        case 0:
          if (tof[CANAL_ARRIERE].present && ar >= 0 && ar < ARRIERE_STOP_MM) moteursStop();
          else reculer(V_SECOUSSE);
          break;
        case 1: tournerGauche(V_SECOUSSE); break;
        default: tournerDroite(V_SECOUSSE); break;
      }
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
    case N_DEGAGE_SCAN:     return "DEGAGE_SCAN";
    case N_DEGAGE_SECOUSSE: return "DEGAGE_SECOUSSE";
  }
  return "?";
}
