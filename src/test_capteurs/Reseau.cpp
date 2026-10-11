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

static void evenementWebSocket(uint8_t client, WStype_t type, uint8_t* donnees, size_t longueur) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[WIFI] client %u connecte (%s)\n", client,
                    webSocket.remoteIP(client).toString().c_str());
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[WIFI] client %u deconnecte\n", client);
      if (rappelDeconnexion) rappelDeconnexion();
      break;
    case WStype_TEXT:
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

void reseauDiffuser(const char* texte, size_t longueur) {
  if (webSocket.connectedClients() > 0)
    webSocket.broadcastTXT((const uint8_t*) texte, longueur);
}

bool reseauConnecte()  { return WiFi.status() == WL_CONNECTED; }
int reseauNbClients()  { return webSocket.connectedClients(); }
int reseauRssi()       { return reseauConnecte() ? WiFi.RSSI() : 0; }
String reseauIp()      { return reseauConnecte() ? WiFi.localIP().toString() : String(""); }
