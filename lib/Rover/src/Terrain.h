/*
 * Terrain.h - etat du sol sous le rover
 * ------------------------------------------------------------
 *   tangage / roulis : pente avant-arriere et gauche-droite (deg),
 *       mesuree par l'accelerometre par rapport au sol plat du depart
 *   rugosite : vibrations verticales (g efficace) ; bosses : chocs
 *       verticaux de plus de 0,3 g comptes depuis le depart
 *   glissement : 0 (adherence) a 1 (patine). Les roues disent qu'il
 *       avance mais la distance a l'obstacle devant ne diminue pas.
 *       Mesure seulement en ligne droite avec un obstacle devant a moins
 *       de 2 m ; sinon "inconnu" (-1).
 */
#pragma once

#include <Arduino.h>

struct InfoTerrain {
  float tangage;        // deg, positif = nez qui monte
  float roulis;         // deg, positif = penche a droite
  float rugosite;       // g efficace
  int bosses;
  float glissement;     // 0..1, -1 = inconnu
};

void terrainCalibrer();          // rover immobile sur sol plat : zero de pente
void terrainMettreAJour();       // a appeler a chaque tour de boucle
void terrainRemettreAZero();     // compteur de bosses
const InfoTerrain& terrainInfo();
