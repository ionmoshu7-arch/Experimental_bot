/*
 * Navigation.h - navigation autonome dans la piece
 * ------------------------------------------------------------
 *   - lignes droites : le gyroscope tient le cap (regulateur PD)
 *   - virages : angle exact au gyroscope, ralentissement a l'approche
 *   - obstacle devant : ralentit, s'arrete, recule si trop pres, puis
 *     tourne vers le cote le plus degage
 *   - mur sur le cote : s'en ecarte doucement
 *   - collision invisible aux capteurs (pied de chaise...) detectee par :
 *       choc a l'accelerometre, cote freine aux encodeurs (rendement
 *       appris en roulant), rover pousse hors de son cap au gyroscope,
 *       virage qui ne tourne pas
 *     -> arret, recul, virage du cote oppose a l'impact ; collisions
 *     repetees : grand recul et demi-tour
 *   - coince (6 collisions en 30 s) : DEGAGEMENT
 *       1. tour complet sur place en mesurant la distance devant tous
 *          les 15 deg, puis depart vers la direction la plus degagee
 *       2. s'il ne peut pas pivoter : essaie dans l'autre sens
 *       3. bloque des deux cotes ou aucune sortie : secousses (recul et
 *          pivots alternes, plus de puissance), puis nouveau tour
 *     Libre apres 3 s sans collision. Arret seulement apres 4 essais.
 *
 * Arret : bouton de la page, STOP, Espace, commande manuelle, page
 * fermee. Demande le gyroscope (cap) et le capteur avant.
 */
#pragma once

#include <Arduino.h>

enum EtatNav { N_ARRET, N_AVANCE, N_PAUSE, N_RECULE, N_TOURNE,
               N_DEGAGE_SCAN, N_DEGAGE_SECOUSSE };

struct InfoNav {
  EtatNav etat;
  float capCible;       // deg, convention : droite = negatif
  int obstacles;
  int collisions;
  int vitesse;          // vitesse de croisiere, sur 255
  const char* evenement;   // dernier evenement, pour l'affichage
  float rendement;      // (mm/s)/consigne appris en roulant, 0 = pas encore
  uint8_t essaiDegagement;   // 0 = pas en degagement, sinon essai en cours
  uint8_t maxEssais;
};

void navigationDemarrer();
void navigationArreter(const char* raison);
void navigationVitesse(int vitesse);
void navigationMettreAJour();     // a appeler a chaque tour de boucle

bool navigationActive();
const InfoNav& navigationInfo();
const char* navigationNomEtat();
