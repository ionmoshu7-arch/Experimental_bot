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
 *     repetees : grand recul et demi-tour, puis arret si coince
 *
 * Arret : bouton de la page, STOP, Espace, commande manuelle, page
 * fermee. Demande le gyroscope (cap) et le capteur avant.
 */
#pragma once

#include <Arduino.h>

enum EtatNav { N_ARRET, N_AVANCE, N_PAUSE, N_RECULE, N_TOURNE };

struct InfoNav {
  EtatNav etat;
  float capCible;       // deg, convention : droite = negatif
  int obstacles;
  int collisions;
  int vitesse;          // vitesse de croisiere, sur 255
  const char* evenement;   // dernier evenement, pour l'affichage
  float rendement;      // (mm/s)/consigne appris en roulant, 0 = pas encore
};

void navigationDemarrer();
void navigationArreter(const char* raison);
void navigationVitesse(int vitesse);
void navigationMettreAJour();     // a appeler a chaque tour de boucle

bool navigationActive();
const InfoNav& navigationInfo();
const char* navigationNomEtat();
