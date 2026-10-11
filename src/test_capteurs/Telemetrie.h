/*
 * Telemetrie.h - sorties sur le port serie
 * ------------------------------------------------------------
 *   - un bloc lisible chaque seconde (moniteur serie de VS Code)
 *   - une ligne "[DATA]{...json...}" 20 fois par seconde, lue par
 *     web/rover_moniteur.html
 */
#pragma once

const unsigned long PERIODE_TEXTE_MS = 1000;   // 0 pour desactiver
const unsigned long PERIODE_JSON_MS = 50;      // 0 pour desactiver

void telemetrieDemarrer();
void telemetrieMettreAJour();   // a appeler a chaque tour de boucle
