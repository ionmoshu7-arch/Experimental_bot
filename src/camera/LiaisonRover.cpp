#include "LiaisonRover.h"

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebSocketsClient.h>
#include "Enregistrement.h"
#include "Journal.h"
#include "Pont.h"
#include "ServeurCamera.h"

static const char* NOM_ROVER = "rover";            // rover.local
static const uint16_t PORT_WEBSOCKET_ROVER = 81;
static const unsigned long PERIODE_ETAT_MS = 1000;
static const unsigned long ATTENTE_RECHERCHE_MS = 5000;

static WebSocketsClient ws;
static bool demarre = false, connectee = false, active = true;
static IPAddress ipRover;
static unsigned long derniereRecherche = 0, dernierEtat = 0, derniereTrame = 0;

static void envoyerEtat() {
  char json[512];
  etatCameraJson(json, sizeof(json));
  String message = String("[CAM]") + json;
  ws.sendTXT(message);
}

static void evenement(WStype_t type, uint8_t* donnees, size_t longueur) {
  switch (type) {
    case WStype_CONNECTED:
      connectee = true;
      Serial.printf("[LIAISON] connectee au rover (%s)\n", ipRover.toString().c_str());
      envoyerEtat();
      break;
    case WStype_DISCONNECTED:
      if (connectee) Serial.println("[LIAISON] rover perdu, reconnexion...");
      connectee = false;
      break;
    case WStype_TEXT:
      // etat du rover : garde pour plus tard (incrustation, IA)
      if (longueur > 6 && !memcmp(donnees, "[DATA]", 6)) {
        derniereTrame = millis();
        enregistrementMesure(donnees + 6, longueur - 6);   // mesures synchronisees
      }
      // ordre d'une mission ou de la page : "[POURCAM] photo"
      if (longueur > 10 && longueur < 64 && !memcmp(donnees, "[POURCAM] ", 10)) {
        char commande[64];
        memcpy(commande, donnees + 10, longueur - 10);
        commande[longueur - 10] = 0;
        pontCommandeCamera(commande);
        break;
      }
      journalMessage(donnees, longueur);                   // journal de navigation
      pontMessageRover(donnees, longueur);                 // console sur le cable USB
      break;
    default:
      break;
  }
}

// Adresse du rover par mDNS (le client WebSocket ne resout pas ".local")
static bool trouverRover() {
  IPAddress ip = MDNS.queryHost(NOM_ROVER, 2000);
  if (ip == IPAddress(0, 0, 0, 0)) return false;
  ipRover = ip;
  return true;
}

void liaisonRoverDemarrer() {
  ws.onEvent(evenement);
  ws.setReconnectInterval(3000);
}

void liaisonRoverActiver(bool a) {
  active = a;
  if (!active && demarre) {
    ws.disconnect();
    demarre = connectee = false;
  }
  Serial.println(active ? "[LIAISON] activee" : "[LIAISON] coupee");
}

bool liaisonRoverActive() { return active; }

void liaisonRoverMettreAJour() {
  if (!active || WiFi.status() != WL_CONNECTED) return;
  unsigned long maintenant = millis();

  if (!demarre) {
    if (maintenant - derniereRecherche < ATTENTE_RECHERCHE_MS && derniereRecherche) return;
    derniereRecherche = maintenant;
    if (!trouverRover()) {
      Serial.println("[LIAISON] rover.local introuvable, nouvel essai dans 5 s");
      return;
    }
    Serial.printf("[LIAISON] rover trouve : %s\n", ipRover.toString().c_str());
    ws.begin(ipRover, PORT_WEBSOCKET_ROVER, "/");
    demarre = true;
  }

  ws.loop();

  // longtemps sans rover : son adresse a peut-etre change, on la recherche
  static unsigned long debutAbsence = 0;
  if (connectee) {
    debutAbsence = 0;
  } else if (!debutAbsence) {
    debutAbsence = maintenant;
  } else if (maintenant - debutAbsence > 20000) {
    ws.disconnect();
    demarre = false;
    debutAbsence = 0;
    return;
  }

  if (connectee && maintenant - dernierEtat >= PERIODE_ETAT_MS) {
    dernierEtat = maintenant;
    envoyerEtat();
  }
}

bool liaisonRoverConnectee() { return connectee; }

bool liaisonRoverEnvoyer(const char* texte) {
  if (!connectee) return false;
  return ws.sendTXT(texte);
}

IPAddress liaisonRoverIp() { return ipRover; }
unsigned long liaisonRoverDerniereTrame() { return derniereTrame; }
