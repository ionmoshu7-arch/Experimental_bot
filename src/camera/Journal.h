/*
 * Journal.h - journal de navigation du rover sur la carte TF de la camera
 * ------------------------------------------------------------
 * Toujours actif quand une carte TF est presente. Chaque message recu du
 * rover est ecrit avec son heure (ms depuis le demarrage de la camera) :
 *     <ms><TAB>[DATA]{...}      etat complet du rover, 5 fois par seconde
 *     <ms><TAB>[NAV] obstacle   evenements (navigation, ligne, missions...)
 * Sert a analyser les trajets, les decisions, et a calibrer les capteurs.
 *
 * Fichiers /journal/J<demarrage>_<n>.jsonl (demarrage = compteur garde en
 * memoire, la carte n'a pas d'horloge) : un nouveau fichier toutes les
 * 10 minutes ; les plus anciens sont effaces au-dela de 48 h ou de 2 Go.
 */
#pragma once

#include <Arduino.h>

constexpr unsigned long DUREE_FICHIER_JOURNAL_MS = 10UL * 60 * 1000;
constexpr int NB_FICHIERS_JOURNAL_MAX = 288;                 // 48 h
constexpr uint64_t TAILLE_JOURNAL_MAX = 2ULL * 1024 * 1024 * 1024;

void journalInit();                         // apres stockageInit()
void journalMessage(const uint8_t* texte, size_t longueur);   // message du rover
void journalMettreAJour();                  // a chaque tour : vidage, rotation
int journalVider();                         // efface tout le journal
void journalListeJson(String &sortie);      // [{"f":"/journal/...","ko":12},...]
const char* journalFichierCourant();
