/*
 * Missions.h - missions autonomes, au-dessus de la navigation
 * ------------------------------------------------------------
 * Commande "M ..." (boutons de la page) :
 *   M explorer <min>   parcourt la piece en evitant les obstacles (la carte
 *                      se remplit dans la page), arret apres <min> minutes
 *   M base             revient a l'origine (0, 0) de la carte, puis se
 *                      tourne vers le cap de depart
 *   M point            memorise la position actuelle (patrouille, 8 max)
 *   M effacer          oublie les points de patrouille
 *   M patrouille       parcourt les points memorises en boucle
 *   M photos 1|0       photo par la camera a chaque point de patrouille
 *   M mur g|d          longe le mur gauche ou droit (tour de la piece)
 *   M suivre <objet>   suit une personne ("person") ou un objet : confie a
 *                      l'IA du PC, qui doit etre lancee
 *   M stop             arrete la mission
 * La camera regarde droit devant : les missions en tiennent compte. Un
 * support 2 axes (PCA9685 #1, canaux 1-2) est prevu plus tard.
 */
#pragma once

#include <Arduino.h>

enum Mission : uint8_t { MI_AUCUNE, MI_EXPLORER, MI_BASE, MI_PATROUILLE, MI_MUR, MI_SUIVRE };

constexpr int MAX_POINTS_PATROUILLE = 8;

struct InfoMission {
  Mission mission;
  const char* etat;          // texte court pour la page
  int point;                 // point de patrouille vise
  int nbPoints;
  long resteS;               // explorer : secondes restantes
  bool photos;               // patrouille : photo a chaque point
  char cible[24];            // suivre : classe YOLO
};

void missionCommande(const char* arguments);   // texte apres "M"
void missionArreter(const char* raison);
void missionsMettreAJour();                    // a chaque tour de boucle
bool missionActive();
bool missionPiloteeParIA();                    // mission "suivre"
const InfoMission& missionInfo();
const char* missionNom();
