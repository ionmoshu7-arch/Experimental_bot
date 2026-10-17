/*
 * RelaisMaj.h - programmer le rover a travers la camera
 * ------------------------------------------------------------
 * Le PC envoie le programme du rover sur le cable USB de la camera
 * (scripts/maj_rover_par_camera.py, env test_capteurs_via_camera) :
 *   PC  : "!MAJROVER <taille> <crc32 hexa>\n"
 *   CAM : "[MAJ] PRET"            (tampon reserve en PSRAM)
 *   PC  : blocs de 4096 octets, chacun suivi par
 *   CAM : "[MAJ] ACK <total>"     (controle de flux)
 *   CAM : "[MAJ] CRC OK" puis envoi HTTP au rover (POST /maj, mot de passe)
 *   CAM : "[MAJ] OK" ou "[MAJ] ERREUR ..."
 * Le rover verifie le mot de passe, s'arrete, s'ecrit et redemarre.
 */
#pragma once

#include <Arduino.h>

constexpr size_t TAILLE_BLOC_MAJ = 4096;

void relaisMajInit(const char* motDePasseOta);
void relaisMajRecevoir(const char* arguments);   // "<taille> <crc32>" ; bloquant
bool relaisMajEnCours();
