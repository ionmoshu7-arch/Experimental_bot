/*
 * Pont.h - console des deux cartes sur le cable USB de la camera
 * ------------------------------------------------------------
 * Moniteur serie de la camera (460800 bauds) :
 *   - les messages du rover s'affichent, prefixes "[ROVER] " ; ses trames
 *     [DATA] sont resumees toutes les 2 s (position, distances, modes)
 *   - une ligne tapee est envoyee au rover : s, c, z, O, T, L 1, N 1, A ...
 *   - une ligne qui commence par "cam " est pour la camera :
 *       cam photo | cam video 1 / 0 | cam res VGA | cam etat
 *       cam liaison 1 / 0 | cam journal | cam vider oui | cam aide
 *   - "!MAJROVER ..." : programme du rover envoye par le PC (RelaisMaj.h)
 */
#pragma once

#include <Arduino.h>

constexpr unsigned long VITESSE_SERIE_CAMERA = 460800;

void pontDemarrer();
void pontMettreAJour();                                  // a chaque tour de boucle
void pontMessageRover(const uint8_t* texte, size_t longueur);   // appele par la liaison
// Commande pour la camera (sans le prefixe "cam ") : photo, video 1, etat...
// Aussi utilisee pour les ordres du rover "[POURCAM] photo" (patrouille).
void pontCommandeCamera(const char* commande);
