/*
 * ServeurCamera.h - serveurs HTTP de la carte camera
 * ------------------------------------------------------------
 *   http://camera.local/          petite page de test
 *   http://camera.local/capture   une image JPEG
 *   http://camera.local/photo     image enregistree sur la carte TF (JSON)
 *   http://camera.local/sd?f=/photos/IMG_0001.jpg   lire une photo
 *   http://camera.local/status    etat en JSON
 *   http://camera.local/control?var=res&val=VGA     reglages :
 *        res (QVGA..UXGA), qualite (10..63), vflip (0/1), miroir (0/1)
 *   http://camera.local:81/stream video MJPEG (un seul ecran a la fois)
 *
 * Toutes les reponses autorisent les autres origines (CORS) : la page
 * du rover (rover.local) peut piloter la camera directement.
 */
#pragma once

#include <Arduino.h>

constexpr uint16_t PORT_HTTP = 80;
constexpr uint16_t PORT_FLUX = 81;

void serveurCameraDemarrer();
int serveurNbFlux();                       // ecrans qui regardent la video
void etatCameraJson(char* buf, size_t taille);
