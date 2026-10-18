/*
 * Stockage.h - carte TF (microSD) de la carte camera, bus SD_MMC 1 bit
 * ------------------------------------------------------------
 * Photos enregistrees dans /photos/IMG_0001.jpg, IMG_0002.jpg...
 * Sans carte, tout fonctionne sauf l'enregistrement.
 */
#pragma once

#include <Arduino.h>

bool stockageInit();                       // false : pas de carte
bool stockagePresent();
int stockageNbPhotos();
uint32_t stockageLibreMo();
// Enregistre une image JPEG ; renvoie le chemin, ou "" en cas d'echec
String stockageEnregistrerPhoto(const uint8_t* jpeg, size_t taille);
// Efface tous les fichiers d'un dossier de la carte ; renvoie leur nombre
int stockageViderDossier(const char* dossier);
// Efface les photos et remet leur numerotation a 1
int stockageViderPhotos();
