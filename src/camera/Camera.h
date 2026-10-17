/*
 * Camera.h - capteur OV2640 de la carte Diymore ESP32-S3-WROOM CAM
 * ------------------------------------------------------------
 * Plusieurs brochages connus sont essayes au demarrage (Freenove /
 * ESP32-S3-EYE, XIAO Sense, T-Camera S3) : le premier qui repond est garde.
 * Images JPEG en PSRAM (2 tampons, toujours la plus recente).
 */
#pragma once

#include <Arduino.h>
#include "esp_camera.h"

bool cameraInit();                       // false si la camera ne repond pas
const char* cameraModele();              // "OV2640", "OV5640"...
const char* cameraBrochage();            // carte reconnue

bool cameraResolution(const char* nom);  // "QVGA", "VGA", "SVGA", "XGA", "HD", "SXGA", "UXGA"
const char* cameraNomResolution();
bool cameraQualite(int q);               // 10 (meilleure) .. 63
int cameraQualiteActuelle();
void cameraRetourner(bool vertical, bool miroir);
bool cameraRetourneeV();
bool cameraMiroir();

// Rotation d'affichage (0, 90, 180, 270 deg dans le sens horaire), selon
// le montage de la camera sur le rover. Le capteur ne sait pas tourner
// l'image : les ecrans l'appliquent. Gardee en memoire (NVS).
void cameraChargerRotation();
bool cameraRotation(int degres);
int cameraRotationActuelle();

// Images par seconde envoyees au flux (mesure)
void cameraCompterImage();
float cameraImagesParSeconde();
