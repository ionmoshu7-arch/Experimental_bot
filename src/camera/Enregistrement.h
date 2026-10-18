/*
 * Enregistrement.h - enregistrement video a la demande sur la carte TF
 * ------------------------------------------------------------
 * Fichiers AVI (MJPEG) : /videos/VID_0001.avi, VID_0002.avi...
 * Lisibles avec VLC ou le lecteur de Windows (pas dans le navigateur).
 *
 * Mesures synchronisees : pendant l'enregistrement, chaque trame [DATA]
 * du rover (position, pente, distances, encodeurs, gyroscope...) est
 * ecrite dans /videos/VID_0001.jsonl, une ligne par trame :
 *     <ms depuis le debut de la video><TAB>{...json du rover...}
 * Le rover en envoie 10 par seconde pendant l'enregistrement.
 *
 * Jamais permanent : demarre et s'arrete sur demande (bouton de la page),
 * et s'arrete seul apres DUREE_MAX_S, si la carte est presque pleine
 * ou en cas d'erreur d'ecriture.
 * Tourne dans sa propre tache : la video en direct continue en parallele
 * (les deux se partagent les images).
 */
#pragma once

#include <Arduino.h>

constexpr int DUREE_MAX_S = 600;           // 10 minutes au plus par fichier
constexpr int IMAGES_PAR_SECONDE_MAX = 10;

void enregistrementInit();                 // apres stockageInit() ; recompte les videos
bool enregistrementDemarrer();             // false : pas de carte, deja en cours...
void enregistrementArreter();
bool enregistrementEnCours();
int enregistrementSecondes();              // duree du fichier en cours
const char* enregistrementFichier();       // dernier fichier (en cours ou termine)
// Trame du rover recue pendant l'enregistrement : ajoutee aux mesures
void enregistrementMesure(const uint8_t* json, size_t longueur);
int enregistrementNbVideos();
