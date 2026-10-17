#include "Reseau.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WebSocketsServer.h>

static WebServer serveur(PORT_WEB);
static WebSocketsServer webSocket(PORT_WEBSOCKET);

static const char* pageControle = "";
static const char* motDePasseOta = nullptr;
static bool servicesDemarres = false;   // mDNS + OTA, apres la 1re connexion
static bool etaitConnecte = false;

static RappelTexte rappelMessage = nullptr;
static RappelSimple rappelDeconnexion = nullptr;
static RappelSimple rappelOta = nullptr;

// La carte camera est un client comme la page, mais elle s'annonce par
// "[CAM]{...}" : son etat est relaye a la page (et au port serie), et
// son depart n'arrete pas les moteurs.
static bool estCamera[WEBSOCKETS_SERVER_CLIENT_MAX] = { false };

static void evenementWebSocket(uint8_t client, WStype_t type, uint8_t* donnees, size_t longueur) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[WIFI] client %u connecte (%s)\n", client,
                    webSocket.remoteIP(client).toString().c_str());
      break;
    case WStype_DISCONNECTED:
      if (client < WEBSOCKETS_SERVER_CLIENT_MAX && estCamera[client]) {
        estCamera[client] = false;
        Serial.println("[CAM] camera deconnectee");
        break;
      }
      Serial.printf("[WIFI] client %u deconnecte\n", client);
      if (rappelDeconnexion) rappelDeconnexion();
      break;
    case WStype_TEXT:
      if (longueur >= 5 && !memcmp(donnees, "[CAM]", 5)) {
        if (client < WEBSOCKETS_SERVER_CLIENT_MAX && !estCamera[client]) {
          estCamera[client] = true;
          Serial.printf("[CAM] camera connectee (%s)\n", webSocket.remoteIP(client).toString().c_str());
        }
        Serial.write(donnees, longueur);          // pour la page connectee par USB
        Serial.println();
        for (uint8_t i = 0; i < WEBSOCKETS_SERVER_CLIENT_MAX; i++)
          if (i != client && !estCamera[i]) webSocket.sendTXT(i, donnees, longueur);
        break;
      }
      if (rappelMessage) rappelMessage((const char*) donnees, longueur);
      break;
    default:
      break;
  }
}

static void demarrerServices() {
  ArduinoOTA.setHostname(NOM_HOTE);     // demarre aussi mDNS : rover.local
  ArduinoOTA.setPassword(motDePasseOta);
  ArduinoOTA.onStart([]() {
    if (rappelOta) rappelOta();
    Serial.println("[OTA] mise a jour en cours...");
  });
  ArduinoOTA.onEnd([]() { Serial.println("[OTA] termine, redemarrage"); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("[OTA] erreur %u\n", e); });
  ArduinoOTA.begin();
  MDNS.addService("http", "tcp", PORT_WEB);
  servicesDemarres = true;
}

void reseauDemarrer(const char* ssid, const char* motDePasse,
                    const char* motDePasseOtaRecu, const char* page) {
  pageControle = page;
  motDePasseOta = motDePasseOtaRecu;

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(NOM_HOTE);
  WiFi.setSleep(false);          // latence plus faible pour le pilotage
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, motDePasse);

  serveur.on("/", []() {
    serveur.send_P(200, "text/html; charset=utf-8", pageControle, strlen(pageControle));
  });
  serveur.onNotFound([]() { serveur.send(404, "text/plain", "introuvable"); });
  serveur.begin();

  webSocket.begin();
  webSocket.onEvent(evenementWebSocket);

  // Attente courte : sans reseau, le rover demarre quand meme
  Serial.printf("[WIFI] connexion a \"%s\"", ssid);
  unsigned long debut = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - debut < 10000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED)
    Serial.println("[WIFI] pas de reseau pour l'instant, nouvel essai en continu");
  reseauMettreAJour();
}

void reseauMettreAJour() {
  bool connecte = WiFi.status() == WL_CONNECTED;
  if (connecte && !etaitConnecte) {
    Serial.printf("[WIFI] connecte : http://%s.local/  ou  http://%s/  (%d dBm)\n",
                  NOM_HOTE, WiFi.localIP().toString().c_str(), WiFi.RSSI());
    if (!servicesDemarres) demarrerServices();
  } else if (!connecte && etaitConnecte) {
    Serial.println("[WIFI] connexion perdue, reconnexion automatique...");
  }
  etaitConnecte = connecte;

  if (servicesDemarres) ArduinoOTA.handle();
  serveur.handleClient();
  webSocket.loop();
}

void reseauSurMessage(RappelTexte rappel)       { rappelMessage = rappel; }
void reseauSurDeconnexion(RappelSimple rappel)  { rappelDeconnexion = rappel; }
void reseauSurOta(RappelSimple rappel)          { rappelOta = rappel; }

// Les pages recoivent tout (20 trames/s). La camera ne recoit qu'une
// trame [DATA] sur 10 (2/s) : a 20/s, son WiFi saturait et la video
// tombait a moins d'une image par seconde.
static const unsigned long PERIODE_DATA_CAMERA_MS = 500;
static unsigned long dernierDataCamera = 0;

void reseauDiffuser(const char* texte, size_t longueur) {
  if (webSocket.connectedClients() == 0) return;
  bool estData = longueur >= 6 && !memcmp(texte, "[DATA]", 6);
  bool pourCamera = !estData || millis() - dernierDataCamera >= PERIODE_DATA_CAMERA_MS;
  if (estData && pourCamera) dernierDataCamera = millis();
  for (uint8_t i = 0; i < WEBSOCKETS_SERVER_CLIENT_MAX; i++) {
    if (estCamera[i] && !pourCamera) continue;
    webSocket.sendTXT(i, (const uint8_t*) texte, longueur);   // ignore si client absent
  }
}

bool reseauConnecte()  { return WiFi.status() == WL_CONNECTED; }
int reseauNbClients()  { return webSocket.connectedClients(); }
int reseauRssi()       { return reseauConnecte() ? WiFi.RSSI() : 0; }
String reseauIp()      { return reseauConnecte() ? WiFi.localIP().toString() : String(""); }
