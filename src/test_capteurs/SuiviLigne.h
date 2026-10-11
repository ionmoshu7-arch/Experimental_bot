/*
 * SuiviLigne.h - suivi d'une ligne noire avec les 3 TCS34725 du sol
 * ------------------------------------------------------------
 * Chaque capteur donne une "noirceur" de 0 (sol) a 1 (ligne), d'apres
 * sa calibration. La position de la ligne va de -1 (sous le capteur
 * gauche) a +1 (sous le capteur droit) ; un regulateur PD corrige
 * l'ecart en ralentissant le cote vers lequel la ligne s'echappe.
 *
 * Calibration : pendant 4 s, faire glisser le rover a la main de part
 * et d'autre de la ligne. Calibration et reglages sont gardes en
 * memoire (NVS) et survivent aux redemarrages.
 *
 * Ligne perdue : le rover continue tout droit 0,15 s (petites coupures),
 * puis, si la recherche est activee, applique une strategie :
 *   PIVOT     pivote vers le cote ou la ligne a ete vue en dernier
 *   RECUL     recule de 10 cm puis pivote d'un cote puis de l'autre
 *   BALAYAGE  regarde a gauche et a droite, angles croissants (gyroscope)
 *   TOUTES    pivot, recul puis balayage, enchaines
 * Sans recherche, ou si elle echoue (12 s max), le rover s'arrete.
 * Pause si un obstacle est a moins de 15 cm devant.
 */
#pragma once

#include <Arduino.h>

enum EtatLigne { L_ARRET, L_SUIT, L_PERDUE, L_RECHERCHE, L_OBSTACLE, L_CALIBRATION };
enum StrategieRecherche : uint8_t { R_PIVOT = 0, R_RECUL = 1, R_BALAYAGE = 2, R_TOUTES = 3 };

struct InfoLigne {
  EtatLigne etat;
  float position;       // -1 gauche .. +1 droite
  float noirceur[3];    // gauche, centre, droite : 0 sol .. 1 ligne
  bool calibre;
  float kp, kd;
  int vitesse;          // vitesse de base, sur 255
  bool recherche;       // rechercher la ligne si elle est perdue
  uint8_t strategie;    // StrategieRecherche
  uint8_t etape;        // etape de recherche en cours
  uint8_t nbEtapes;
};

void suiviLigneInit();                 // charge calibration et reglages
void suiviLigneDemarrer();
void suiviLigneArreter(const char* raison);
void suiviLigneCalibrer();             // lance les 4 s de calibration
void suiviLigneReglages(float kp, float kd, int vitesse);
void suiviLigneRecherche(bool active, int strategie);
void suiviLigneMettreAJour();          // a appeler a chaque tour de boucle

bool suiviLigneOccupe();               // suivi ou calibration en cours
const InfoLigne& suiviLigneInfo();
const char* suiviLigneNomEtat();
const char* suiviLigneNomStrategie(uint8_t strategie);
