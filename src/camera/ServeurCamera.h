/*
 * ServeurCamera.h - serveurs HTTP de la carte camera
 * ------------------------------------------------------------
 *   http://camera.local/          petite page de test
 *   http://camera.local/capture   une image JPEG
 *   http://camera.local/photo     image enregistree sur la carte TF (JSON)
 *   http://camera.local/sd?f=/photos/IMG_0001.jpg   lire une photo
 *   http://camera.local/status    etat en JSON
 *   http://camera.local/control?var=res&val=VGA     reglages :
 *        res (QVGA..UXGA), qualite (10..63), vflip (0/1), miroir (0/1),
 *        rotation (0/90/180/270, appliquee par les ecrans),
 *        video (1 demarre / 0 arrete l'enregistrement sur la carte TF)
 *   http://camera.local/sd?f=/videos/VID_0001.avi   telecharger une video
 *   http://camera.local:81/stream video MJPEG (un seul ecran a la fois)
 *   POST http://camera.local/maj   nouveau programme (en-tete X-Mot-De-Passe)
 *   http://camera.local/journaux   liste du journal de navigation (JSON)
 *   http://camera.local/vider?confirmer=oui   efface photos, videos et journal
 *
 * Toutes les reponses autorisent les autres origines (CORS) : la page
 * du rover (rover.local) peut piloter la camera directement.
 */
#pragma once

#include <Arduino.h>

constexpr uint16_t PORT_HTTP = 80;
constexpr uint16_t PORT_FLUX = 81;
// 2e flux video, pour l'IA du PC : chaque serveur ne sert qu'un ecran a la
// fois (la boucle MJPEG occupe sa tache), la page garde ainsi le port 81.
constexpr uint16_t PORT_FLUX_IA = 82;

void serveurCameraDemarrer(const char* motDePasseMaj);   // mot de passe de /maj
int serveurNbFlux();                       // ecrans qui regardent la video
void etatCameraJson(char* buf, size_t taille);
int viderCarte();                          // photos, videos, journal ; -1 si occupee
