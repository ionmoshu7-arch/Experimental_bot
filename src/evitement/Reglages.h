/*
 * Reglages.h - parametres de l'evitement d'obstacles
 */
#pragma once

const unsigned long VITESSE_SERIE = 115200;

const int VITESSE_AVANCE = 100;      // sur 255
const int VITESSE_ROTATION = 110;
const int VITESSE_RECUL = 100;

const int SEUIL_OBSTACLE_MM = 250;   // declenchement avant
const int SEUIL_ARRIERE_MM = 180;    // distance minimale derriere
const float RECUL_CM = 12.0;
const float ANGLE_VIRAGE_DEG = 70.0;

const int COMPTE_A_REBOURS_S = 10;
const int DUREE_MAX_MIN = 5;                   // 0 pour desactiver
const unsigned long DEBUG_PERIODE_MS = 500;    // 0 pour desactiver
