#include "ServeurCamera.h"

#include <WiFi.h>
#include <FS.h>
#include <SD_MMC.h>
#include "esp_http_server.h"
#include "Camera.h"
#include "LiaisonRover.h"
#include "Stockage.h"

#define LIMITE "rover-camera-limite"
static const char* TYPE_FLUX = "multipart/x-mixed-replace;boundary=" LIMITE;
static const char* SEPARATEUR = "\r\n--" LIMITE "\r\n";
static const char* ENTETE_IMAGE = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t serveurHttp = nullptr, serveurFlux = nullptr;
static volatile int nbFlux = 0;

int serveurNbFlux() { return nbFlux; }

void etatCameraJson(char* buf, size_t taille) {
  String ip = WiFi.localIP().toString();
  snprintf(buf, taille,
           "{\"ip\":\"%s\",\"flux\":\"http://%s:%u/stream\",\"modele\":\"%s\","
           "\"res\":\"%s\",\"qualite\":%d,\"vflip\":%d,\"miroir\":%d,\"ips\":%.1f,"
           "\"ecrans\":%d,\"sd\":%d,\"photos\":%d,\"libreMo\":%u,\"rssi\":%d,"
           "\"rover\":%d,\"psramLibreKo\":%u}",
           ip.c_str(), ip.c_str(), PORT_FLUX, cameraModele(), cameraNomResolution(),
           cameraQualiteActuelle(), cameraRetourneeV(), cameraMiroir(),
           cameraImagesParSeconde(), nbFlux, stockagePresent(), stockageNbPhotos(),
           stockageLibreMo(), WiFi.RSSI(), liaisonRoverConnectee(),
           (unsigned) (ESP.getFreePsram() / 1024));
}

static void autoriserOrigines(httpd_req_t* req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
}

static esp_err_t envoyerJson(httpd_req_t* req, const char* json) {
  autoriserOrigines(req);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

// ---------- page de test ----------
static esp_err_t pageAccueil(httpd_req_t* req) {
  String ip = WiFi.localIP().toString();
  String page =
    "<!doctype html><html lang=fr><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>Camera rover</title><body style='font-family:system-ui;margin:16px'>"
    "<h1 style='font-size:20px'>Camera du rover</h1>"
    "<img src='http://" + ip + ":81/stream' style='max-width:100%;border-radius:8px'>"
    "<p><a href='/capture'>Image</a> &middot; <a href='/photo'>Photo sur la carte TF</a>"
    " &middot; <a href='/status'>Etat</a></p>"
    "<p>Page complete : celle du rover (http://rover.local/).</p></body></html>";
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, page.c_str(), page.length());
}

// ---------- une image ----------
static esp_err_t capture(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
  autoriserOrigines(req);
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  esp_err_t res = httpd_resp_send(req, (const char*) fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// ---------- photo sur la carte TF ----------
static esp_err_t photo(httpd_req_t* req) {
  char json[160];
  if (!stockagePresent()) return envoyerJson(req, "{\"ok\":0,\"erreur\":\"pas de carte TF\"}");
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return envoyerJson(req, "{\"ok\":0,\"erreur\":\"pas d'image\"}");
  String chemin = stockageEnregistrerPhoto(fb->buf, fb->len);
  size_t taille = fb->len;
  esp_camera_fb_return(fb);
  if (chemin.isEmpty()) return envoyerJson(req, "{\"ok\":0,\"erreur\":\"ecriture impossible\"}");
  Serial.printf("[CAM] photo %s (%u octets)\n", chemin.c_str(), (unsigned) taille);
  snprintf(json, sizeof(json), "{\"ok\":1,\"fichier\":\"%s\",\"octets\":%u}",
           chemin.c_str(), (unsigned) taille);
  return envoyerJson(req, json);
}

// ---------- lire un fichier de la carte TF ----------
static esp_err_t lireFichier(httpd_req_t* req) {
  char requete[96], chemin[64];
  if (httpd_req_get_url_query_str(req, requete, sizeof(requete)) != ESP_OK ||
      httpd_query_key_value(requete, "f", chemin, sizeof(chemin)) != ESP_OK ||
      strncmp(chemin, "/photos/", 8) != 0 || strstr(chemin, "..")) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  File f = SD_MMC.open(chemin);
  if (!f) { httpd_resp_send_404(req); return ESP_FAIL; }
  autoriserOrigines(req);
  httpd_resp_set_type(req, "image/jpeg");
  uint8_t tampon[2048];
  size_t n;
  while ((n = f.read(tampon, sizeof(tampon))) > 0) {
    if (httpd_resp_send_chunk(req, (const char*) tampon, n) != ESP_OK) break;
  }
  f.close();
  return httpd_resp_send_chunk(req, nullptr, 0);
}

// ---------- etat ----------
static esp_err_t status(httpd_req_t* req) {
  char json[512];
  etatCameraJson(json, sizeof(json));
  return envoyerJson(req, json);
}

// ---------- reglages ----------
static esp_err_t controle(httpd_req_t* req) {
  char requete[96], var[16], val[16];
  if (httpd_req_get_url_query_str(req, requete, sizeof(requete)) != ESP_OK ||
      httpd_query_key_value(requete, "var", var, sizeof(var)) != ESP_OK ||
      httpd_query_key_value(requete, "val", val, sizeof(val)) != ESP_OK) {
    return envoyerJson(req, "{\"ok\":0,\"erreur\":\"var et val attendus\"}");
  }
  bool ok = true;
  if (!strcmp(var, "res"))          ok = cameraResolution(val);
  else if (!strcmp(var, "qualite")) ok = cameraQualite(atoi(val));
  else if (!strcmp(var, "vflip"))   cameraRetourner(atoi(val), cameraMiroir());
  else if (!strcmp(var, "miroir"))  cameraRetourner(cameraRetourneeV(), atoi(val));
  else ok = false;
  Serial.printf("[CAM] reglage %s = %s : %s\n", var, val, ok ? "ok" : "refuse");
  return status(req);
}

// ---------- flux video MJPEG ----------
static esp_err_t flux(httpd_req_t* req) {
  esp_err_t res = httpd_resp_set_type(req, TYPE_FLUX);
  if (res != ESP_OK) return res;
  autoriserOrigines(req);
  nbFlux++;
  Serial.println("[CAM] un ecran regarde la video");
  char entete[64];
  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    size_t lg = snprintf(entete, sizeof(entete), ENTETE_IMAGE, (unsigned) fb->len);
    res = httpd_resp_send_chunk(req, SEPARATEUR, strlen(SEPARATEUR));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, entete, lg);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*) fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;     // ecran ferme
    cameraCompterImage();
  }
  nbFlux--;
  Serial.println("[CAM] ecran video ferme");
  return res;
}

void serveurCameraDemarrer() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = PORT_HTTP;
  cfg.ctrl_port = 32768;
  cfg.max_uri_handlers = 8;
  if (httpd_start(&serveurHttp, &cfg) == ESP_OK) {
    httpd_uri_t pages[] = {
      { "/",        HTTP_GET, pageAccueil, nullptr },
      { "/capture", HTTP_GET, capture,     nullptr },
      { "/photo",   HTTP_GET, photo,       nullptr },
      { "/sd",      HTTP_GET, lireFichier, nullptr },
      { "/status",  HTTP_GET, status,      nullptr },
      { "/control", HTTP_GET, controle,    nullptr },
    };
    for (auto &p : pages) httpd_register_uri_handler(serveurHttp, &p);
  }

  httpd_config_t cfgFlux = HTTPD_DEFAULT_CONFIG();
  cfgFlux.server_port = PORT_FLUX;
  cfgFlux.ctrl_port = 32769;
  if (httpd_start(&serveurFlux, &cfgFlux) == ESP_OK) {
    httpd_uri_t uriFlux = { "/stream", HTTP_GET, flux, nullptr };
    httpd_register_uri_handler(serveurFlux, &uriFlux);
  }
  Serial.printf("[CAM] serveurs : http %u, video %u\n", PORT_HTTP, PORT_FLUX);
}
