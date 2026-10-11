/*
 * Reseau.h - WiFi, page de controle, WebSocket et mises a jour OTA
 * ------------------------------------------------------------
 *   http://rover.local/       page de controle (servie par le rover)
 *   ws://rover.local:81/      donnees [DATA] et commandes, comme l'USB
 *   OTA "rover"               televersement par WiFi (env test_capteurs_ota)
 *
 * La connexion WiFi ne bloque jamais le rover : s'il n'y a pas de
 * reseau, tout fonctionne par USB et le WiFi se reconnecte seul.
 */
#pragma once

#include <Arduino.h>

constexpr const char* NOM_HOTE = "rover";
constexpr uint16_t PORT_WEB = 80;
constexpr uint16_t PORT_WEBSOCKET = 81;

using RappelTexte = void (*)(const char* texte, size_t longueur);
using RappelSimple = void (*)();

// page : contenu de la page de controle (fichier embarque, termine par 0)
void reseauDemarrer(const char* ssid, const char* motDePasse,
                    const char* motDePasseOta, const char* page);
void reseauMettreAJour();          // a appeler a chaque tour de boucle

void reseauSurMessage(RappelTexte rappel);       // texte recu d'un client
void reseauSurDeconnexion(RappelSimple rappel);  // un client est parti
void reseauSurOta(RappelSimple rappel);          // debut de mise a jour

void reseauDiffuser(const char* texte, size_t longueur);   // vers tous les clients
bool reseauConnecte();
int reseauNbClients();
int reseauRssi();
String reseauIp();
