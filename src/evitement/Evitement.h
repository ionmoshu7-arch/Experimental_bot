/*
 * Evitement.h - machine a etats de l'evitement d'obstacles
 */
#pragma once

enum Etat {
  E_AVANCE,
  E_ARRET_COMPLET,
  E_VERIFIE_ARRIERE,
  E_RECULE,
  E_MESURE_COTES,
  E_TOURNE,
  E_FIN
};

void evitementDemarrer();     // active les moteurs et part en avant
void evitementPas();          // un tour de la machine a etats

Etat evitementEtat();
const char* evitementNomEtat();
float evitementCapCible();
int evitementObstacles();
int evitementReculsAnnules();
