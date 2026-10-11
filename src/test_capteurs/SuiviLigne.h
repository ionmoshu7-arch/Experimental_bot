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
 * Arret automatique : ligne perdue plus de 1,5 s. Pause si un
 * obstacle est a moins de 15 cm devant, reprise au-dela de 22 cm.
 */
#pragma once

#include <Arduino.h>

enum EtatLigne { L_ARRET, L_SUIT, L_PERDUE, L_OBSTACLE, L_CALIBRATION };

struct InfoLigne {
  EtatLigne etat;
  float position;       // -1 gauche .. +1 droite
  float noirceur[3];    // gauche, centre, droite : 0 sol .. 1 ligne
  bool calibre;
  float kp, kd;
  int vitesse;          // vitesse de base, sur 255
};

void suiviLigneInit();                 // charge calibration et reglages
void suiviLigneDemarrer();
void suiviLigneArreter(const char* raison);
void suiviLigneCalibrer();             // lance les 4 s de calibration
void suiviLigneReglages(float kp, float kd, int vitesse);
void suiviLigneMettreAJour();          // a appeler a chaque tour de boucle

bool suiviLigneOccupe();               // suivi ou calibration en cours
const InfoLigne& suiviLigneInfo();
const char* suiviLigneNomEtat();
