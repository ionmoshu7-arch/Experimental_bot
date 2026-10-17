/*
 * RemoteXYControle.h - pilotage depuis l'application RemoteXY (Android/iOS)
 * ------------------------------------------------------------
 * RemoteXY ecoute sur le port 6377 de la connexion WiFi existante
 * (Reseau.cpp) : la page web, le WebSocket et l'OTA continuent de marcher.
 *
 * L'interface (joystick, boutons...) est dessinee sur remotexy.com et
 * son code genere est colle dans src/test_capteurs/RemoteXYInterface.h.
 * Tant que ce fichier n'existe pas, RemoteXY est simplement desactive.
 * Elements attendus : voir docs/RemoteXY.md.
 *
 * Securite : joystick relache -> moteurs arretes ; application
 * deconnectee -> tout s'arrete.
 */
#pragma once

constexpr unsigned short PORT_REMOTEXY = 6377;

void remoteXYDemarrer();        // apres reseauDemarrer()
void remoteXYMettreAJour();     // a appeler a chaque tour de boucle
bool remoteXYDisponible();      // interface presente dans le firmware
bool remoteXYConnecte();        // l'application est connectee
