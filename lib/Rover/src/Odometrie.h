/*
 * Odometrie.h - position du rover (x, y, cap) depuis le depart
 * ------------------------------------------------------------
 * Distance : moyenne des 4 encodeurs (576 impulsions/tour, 0,365 mm).
 * Direction : cap du gyroscope (plus fiable que la difference des roues,
 * qui patinent en virage).
 * Repere : origine = position au demarrage (ou a la remise a zero),
 * x vers l'avant de depart, y vers la gauche, cap en degres
 * (positif = a gauche, convention du projet).
 */
#pragma once

#include <Arduino.h>

struct Pose {
  float x, y;           // mm
  float cap;            // deg
  float distance;       // mm parcourus (avant + arriere)
};

void odometrieInit();              // apres moteursInit() et gyroInit()
void odometrieMettreAJour();       // a appeler a chaque tour de boucle
void odometrieRemettreAZero();     // nouvelle origine ici
// A appeler apres une remise a zero du cap du gyroscope (calibration,
// commande z) : la position garde son orientation, sans saut.
void odometrieRecalerCap();
const Pose& odometriePose();
