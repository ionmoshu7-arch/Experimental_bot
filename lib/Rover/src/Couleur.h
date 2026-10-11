/*
 * Couleur.h - 3 capteurs de couleur TCS34725 sous le rover
 * ------------------------------------------------------------
 *   canal 4 : sol gauche   canal 5 : sol centre   canal 6 : sol droit
 * Pilotes en registres directs, sans bibliotheque.
 * Lecture non bloquante : un capteur n'est lu que si sa mesure est prete.
 */
#pragma once

#include <Arduino.h>

struct CapteurCouleur {
  const char* nom;
  uint8_t canal;
  bool present;
  bool valide;       // au moins une mesure lue
  uint16_t c, r, g, b;
  float lux;
  int cct;           // temperature de couleur en kelvin (0 = inconnue)
  float lum;         // c / comptage maximal, 0 a 1
  bool sature;
  const char* couleur;
};

extern CapteurCouleur couleurs[3];

void couleurInit();
void couleurMettreAJour();
