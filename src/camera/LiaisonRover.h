/*
 * LiaisonRover.h - synchronisation camera <-> rover par le WiFi
 * ------------------------------------------------------------
 * La camera se connecte au WebSocket du rover (ws://rover.local:81),
 * comme la page web :
 *   - elle envoie son etat chaque seconde : "[CAM]{...json...}"
 *     (le rover le relaie a la page, qui trouve ainsi l'adresse video)
 *   - elle recoit l'etat du rover ([DATA] 20 fois/s), memorise pour la
 *     suite (incrustation, reconnaissance d'objets...)
 * Reconnexion automatique si le rover redemarre.
 */
#pragma once

#include <Arduino.h>

void liaisonRoverDemarrer();
void liaisonRoverMettreAJour();     // a appeler a chaque tour de boucle
bool liaisonRoverConnectee();
void liaisonRoverActiver(bool active);      // coupe / retablit la liaison
bool liaisonRoverActive();
unsigned long liaisonRoverDerniereTrame();   // millis() de la derniere [DATA]
