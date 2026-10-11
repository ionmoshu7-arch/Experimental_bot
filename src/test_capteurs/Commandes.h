/*
 * Commandes.h - commandes recues par USB (port serie) ou par WiFi
 * ------------------------------------------------------------
 *   c : recalibrer le gyroscope (ne pas bouger le rover)
 *   z : remettre le cap a zero
 *   s : arret de tous les moteurs
 *   A v1 v2 v3 v4 + Entree : consigne des moteurs M1 M2 M3 M4,
 *       de -255 a 255, positif = vers l'avant (a repeter < 400 ms)
 *
 * Securite : sans commande moteur depuis 400 ms, tout s'arrete.
 * La deconnexion d'un client WiFi arrete aussi les moteurs.
 */
#pragma once

#include <Arduino.h>

const unsigned long DELAI_SECURITE_MOTEURS_MS = 400;

void commandesLire();                                    // port serie
void commandesTraiterWifi(const char* texte, size_t n);  // message WebSocket
void commandesSurveiller();   // arret de securite des moteurs
void arreterMoteurs();        // consignes a 0 et STBY a LOW
