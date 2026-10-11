/*
 * Commandes.h - commandes recues sur le port serie
 * ------------------------------------------------------------
 *   c : recalibrer le gyroscope (ne pas bouger le rover)
 *   z : remettre le cap a zero
 *   s : arret de tous les moteurs
 *   A v1 v2 v3 v4 + Entree : consigne des moteurs M1 M2 M3 M4,
 *       de -255 a 255, positif = vers l'avant (a repeter < 400 ms)
 *
 * Securite : sans commande moteur depuis 400 ms, tout s'arrete.
 */
#pragma once

const unsigned long DELAI_SECURITE_MOTEURS_MS = 400;

void commandesLire();         // a appeler a chaque tour de boucle
void commandesSurveiller();   // arret de securite des moteurs
void arreterMoteurs();        // consignes a 0 et STBY a LOW
