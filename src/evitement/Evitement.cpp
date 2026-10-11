#include "Evitement.h"

#include <Arduino.h>
#include "Config.h"
#include "Distance.h"
#include "Gyro.h"
#include "Moteurs.h"
#include "Reglages.h"

static Etat etat = E_AVANCE;
static float capCible = 0.0;
static unsigned long horodatageEtat = 0;
static unsigned long horodatageDepart = 0;
static bool virageADroite = true;
static int nbObstacles = 0;
static int nbReculsAnnules = 0;

static void imprimerMesure(uint8_t canal) {
  char buf[64];
  distanceTexte(canal, buf, sizeof(buf), true);
  Serial.printf("  %s = %s\n", tof[canal].nom, buf);
}

static void changerEtat(Etat e) {
  etat = e;
  horodatageEtat = millis();
}

static void arretDefinitif(const char* raison) {
  moteursStop();
  moteursActiver(false);
  etat = E_FIN;

  Serial.println();
  Serial.print(">>> ARRET : ");
  Serial.println(raison);
  Serial.print("    Obstacles evites : ");
  Serial.println(nbObstacles);
  Serial.print("    Reculs annules (pas de place) : ");
  Serial.println(nbReculsAnnules);
}

// Mesure les cotes, choisit la direction, fixe le cap cible et
// lance le mouvement.
static void lancerRotation() {
  int dGauche = distanceUtile(CANAL_GAUCHE);
  int dDroite = distanceUtile(CANAL_DROITE);

  imprimerMesure(CANAL_GAUCHE);
  imprimerMesure(CANAL_DROITE);

  if (dGauche > 0 && dDroite > 0) {
    virageADroite = (dDroite > dGauche);
  } else {
    // Sans mesure laterale fiable, on alterne pour eviter de
    // tourner toujours du meme cote et de boucler sur place.
    virageADroite = !virageADroite;
  }

  // Convention validee : rotation a DROITE = cap NEGATIF
  capCible = gyroCap() + (virageADroite ? -ANGLE_VIRAGE_DEG : ANGLE_VIRAGE_DEG);

  if (virageADroite) {
    tournerDroite(VITESSE_ROTATION);
    Serial.println("  rotation a droite");
  } else {
    tournerGauche(VITESSE_ROTATION);
    Serial.println("  rotation a gauche");
  }
  changerEtat(E_TOURNE);
}

void evitementDemarrer() {
  moteursActiver(true);
  moteursRemettreAZero();
  horodatageDepart = millis();
  changerEtat(E_AVANCE);
  avancer(VITESSE_AVANCE);
}

void evitementPas() {
  if (etat == E_FIN) return;

  // --- Duree maximale ---
  if (DUREE_MAX_MIN > 0 &&
      millis() - horodatageDepart > (unsigned long) DUREE_MAX_MIN * 60000UL) {
    arretDefinitif("duree maximale atteinte");
    return;
  }

  int distanceAvant = distanceUtile(CANAL_AVANT);
  if (distanceAvant < 0) {
    arretDefinitif("le capteur avant ne repond plus");
    return;
  }

  switch (etat) {

    case E_AVANCE:
      if (distanceAvant < SEUIL_OBSTACLE_MM) {
        nbObstacles++;
        Serial.printf("Obstacle #%d a %d mm\n", nbObstacles, distanceAvant);
        moteursStop();
        changerEtat(E_ARRET_COMPLET);
      }
      break;

    case E_ARRET_COMPLET:
      // Laisse les roues s'immobiliser avant toute mesure aux
      // encodeurs : l'inertie fausserait le point de depart.
      if (millis() - horodatageEtat > 300) changerEtat(E_VERIFIE_ARRIERE);
      break;

    case E_VERIFIE_ARRIERE: {
      // On regarde derriere AVANT de reculer.
      int dArriere = distanceUtile(CANAL_ARRIERE);
      bool reculPossible;

      if (!tof[CANAL_ARRIERE].present) {
        // Pas de capteur : recul a l'aveugle comme la version 12
        reculPossible = true;
      } else if (dArriere < 0) {
        // Lecture invalide : par prudence, on ne recule pas
        reculPossible = false;
        Serial.println("  arriere : lecture invalide, recul annule");
      } else {
        imprimerMesure(CANAL_ARRIERE);
        // Il faut la place de reculer, plus une marge de securite
        reculPossible = (dArriere > (RECUL_CM * 10 + SEUIL_ARRIERE_MM));
      }

      if (reculPossible) {
        moteursRemettreAZero();
        reculer(VITESSE_RECUL);
        changerEtat(E_RECULE);
      } else {
        // Pas la place : on tourne sur place, sans reculer.
        nbReculsAnnules++;
        Serial.println("  place insuffisante derriere -> rotation directe");
        lancerRotation();
      }
      break;
    }

    case E_RECULE: {
      // Surveillance permanente pendant le recul.
      int dArriere = distanceUtile(CANAL_ARRIERE);
      if (dArriere > 0 && dArriere < SEUIL_ARRIERE_MM) {
        moteursStop();
        Serial.printf("  obstacle arriere a %d mm -> recul interrompu\n", dArriere);
        delay(200);
        lancerRotation();
        break;
      }

      float parcouruCm = (compteMoyen() / IMPULSIONS_PAR_TOUR) * CIRCONFERENCE_CM;

      // Securite : si les encodeurs ne comptent pas, le recul ne
      // se terminerait jamais.
      if (parcouruCm >= RECUL_CM || millis() - horodatageEtat > 3000) {
        moteursStop();
        changerEtat(E_MESURE_COTES);
      }
      break;
    }

    case E_MESURE_COTES:
      if (millis() - horodatageEtat > 300) lancerRotation();
      break;

    case E_TOURNE: {
      float cap = gyroCap();
      bool termine = virageADroite ? (cap <= capCible) : (cap >= capCible);

      // Securite : sans gyroscope, la rotation ne finirait jamais.
      if (termine || (millis() - horodatageEtat > 4000)) {
        moteursStop();
        Serial.printf("  cap : %.1f deg\n\n", cap);
        delay(200);
        moteursRemettreAZero();
        avancer(VITESSE_AVANCE);
        changerEtat(E_AVANCE);
      }
      break;
    }

    default:
      break;
  }
}

Etat evitementEtat()        { return etat; }
float evitementCapCible()   { return capCible; }
int evitementObstacles()    { return nbObstacles; }
int evitementReculsAnnules() { return nbReculsAnnules; }

const char* evitementNomEtat() {
  switch (etat) {
    case E_AVANCE:          return "AVANCE";
    case E_ARRET_COMPLET:   return "ARRET";
    case E_VERIFIE_ARRIERE: return "VERIF_AR";
    case E_RECULE:          return "RECULE";
    case E_MESURE_COTES:    return "MESURE";
    case E_TOURNE:          return "TOURNE";
    case E_FIN:             return "FIN";
  }
  return "?";
}

/*
 * ============================================================
 * PRINCIPE
 * ============================================================
 *   obstacle -> mesure arriere
 *     espace suffisant   -> recul surveille en continu
 *     espace insuffisant -> rotation sur place, sans reculer
 *     obstacle detecte pendant le recul -> arret immediat, rotation
 *
 * Le rover peut ainsi se degager d'un coin sans se coincer.
 *
 * COMPTEURS AFFICHES A L'ARRET
 *   Obstacles evites  : nombre de detections frontales
 *   Reculs annules    : nombre de fois ou la place manquait derriere
 * Un nombre eleve de reculs annules signale un environnement
 * encombre, ou un SEUIL_ARRIERE_MM trop prudent.
 *
 * REGLAGES (Reglages.h)
 *   SEUIL_ARRIERE_MM  marge de securite derriere. Diminuer si le
 *                     rover refuse trop souvent de reculer ;
 *                     augmenter s'il frotte un mur.
 *   ANGLE_VIRAGE_DEG  augmenter s'il retrouve le meme obstacle
 *                     apres avoir tourne.
 *   tof[]             (lib/Rover/src/Distance.cpp) portee et fiabilite
 *                     de chaque capteur (droite : 25 cm seulement).
 *
 * LIMITE CONNUE
 *   4,5 % d'ecart de vitesse entre cotes droit et gauche : le rover
 *   ne roule pas droit. Correction prevue par asservissement des
 *   encodeurs. Les vitesses v_M1..v_M4 du debug mesurent cet ecart.
 * ============================================================
 */
