/*
 * Commandes.h - commandes recues par USB (port serie) ou par WiFi
 * ------------------------------------------------------------
 *   c : recalibrer le gyroscope (ne pas bouger le rover)
 *   z : remettre le cap a zero
 *   s : arret de tous les moteurs
 *   A v1 v2 v3 v4 + Entree : consigne des moteurs M1 M2 M3 M4,
 *       de -255 a 255, positif = vers l'avant (a repeter < 400 ms)
 *   L 1 / L 0 + Entree : demarrer / arreter le suivi de ligne
 *   K + Entree : calibration du suivi de ligne (4 s)
 *   P kp kd vitesse + Entree : reglages du suivi de ligne
 *   R a s + Entree : recherche de ligne perdue, a = 1 active / 0 non,
 *       s = 0 pivot, 1 recul, 2 balayage, 3 toutes
 *   N 1 / N 0 + Entree : demarrer / arreter la navigation autonome
 *   Q vitesse + Entree : vitesse de croisiere de la navigation
 *
 * Securite : sans commande moteur depuis 400 ms, tout s'arrete
 * (sauf en mode autonome, qui a ses propres arrets).
 * Une commande moteur manuelle ou "s" arrete les modes autonomes.
 * La deconnexion d'un client WiFi arrete tout.
 */
#pragma once

#include <Arduino.h>

const unsigned long DELAI_SECURITE_MOTEURS_MS = 400;

void commandesLire();                                    // port serie
void commandesTraiterWifi(const char* texte, size_t n);  // message WebSocket
void commandesSurveiller();   // arret de securite des moteurs
void arreterMoteurs();        // consignes a 0 et STBY a LOW
void arretGeneral();          // arrete le suivi de ligne et les moteurs
