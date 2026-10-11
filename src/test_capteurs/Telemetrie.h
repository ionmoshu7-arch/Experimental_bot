/*
 * Telemetrie.h - sorties vers le port serie et les clients WiFi
 * ------------------------------------------------------------
 *   - un bloc lisible chaque seconde (port serie seulement)
 *   - une ligne "[DATA]{...json...}" 20 fois par seconde, envoyee au
 *     port serie et aux pages connectees par WiFi
 *   - journal() : message court envoye aux deux
 */
#pragma once

#include <Arduino.h>

const unsigned long PERIODE_TEXTE_MS = 1000;   // 0 pour desactiver
const unsigned long PERIODE_JSON_MS = 50;      // 0 pour desactiver

void telemetrieDemarrer();
void telemetrieMettreAJour();   // a appeler a chaque tour de boucle

void journal(const char* format, ...);
