/*
 * Inventaire.h - rapport materiel au demarrage, compare au plan
 * d'adressage I2C et au cablage de 01_Rover_Suivi.xlsx.
 */
#pragma once

// Scan du bus principal et des 8 canaux du mux, WHO_AM_I du MPU-6500,
// cablage moteurs/encodeurs. A appeler apres busInit() et moteursInit().
void inventaireAfficher();

// Etat des capteurs de distance, apres distanceInit()
void inventaireAfficherDistance();

// Etat des capteurs de couleur, apres couleurInit()
void inventaireAfficherCouleur();
