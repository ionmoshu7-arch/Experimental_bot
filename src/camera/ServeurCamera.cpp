#include "ServeurCamera.h"

#include <WiFi.h>
#include <FS.h>
#include <SD_MMC.h>
#include "esp_http_server.h"
#include <Update.h>
#include "Camera.h"
#include "Enregistrement.h"
#include "Journal.h"
#include "RelaisMaj.h"
#include "LiaisonRover.h"
#include "Stockage.h"

#define LIMITE "rover-camera-limite"
static const char* TYPE_FLUX = "multipart/x-mixed-replace;boundary=" LIMITE;
static const char* SEPARATEUR = "\r\n--" LIMITE "\r\n";
static const char* ENTETE_IMAGE = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t serveurHttp = nullptr, serveurFlux = nullptr, serveurFluxIA = nullptr;
static const char* motDePasseMaj = nullptr;
static volatile int nbFlux = 0;

int serveurNbFlux() { return nbFlux; }

void etatCameraJson(char* buf, size_t taille) {
  String ip = WiFi.localIP().toString();
  snprintf(buf, taille,
           "{\"ip\":\"%s\",\"flux\":\"http://%s:%u/stream\",\"fluxIA\":\"http://%s:%u/stream\","
           "\"modele\":\"%s\","
           "\"res\":\"%s\",\"qualite\":%d,\"vflip\":%d,\"miroir\":%d,\"rotation\":%d,\"ips\":%.1f,"
           "\"ecrans\":%d,\"sd\":%d,\"photos\":%d,\"libreMo\":%u,\"rssi\":%d,\"tx\":%.1f,"
           "\"rover\":%d,\"psramLibreKo\":%u,\"rec\":%d,\"recSec\":%d,\"recMax\":%d,"
           "\"recFichier\":\"%s\",\"videos\":%d,\"version\":\"%s %s\"}",
           ip.c_str(), ip.c_str(), PORT_FLUX, ip.c_str(), PORT_FLUX_IA, cameraModele(), cameraNomResolution(),
           cameraQualiteActuelle(), cameraRetourneeV(), cameraMiroir(), cameraRotationActuelle(),
           cameraImagesParSeconde(), nbFlux, stockagePresent(), stockageNbPhotos(),
           stockageLibreMo(), WiFi.RSSI(), WiFi.getTxPower() / 4.0f, liaisonRoverConnectee(),
           (unsigned) (ESP.getFreePsram() / 1024), enregistrementEnCours(),
           enregistrementSecondes(), DUREE_MAX_S, enregistrementFichier(),
           enregistrementNbVideos(), __DATE__, __TIME__);
}

// ---------- controle d'acces ----------
// Protege la video, les photos, le journal et la carte TF des sites Internet
// ouverts dans un navigateur du reseau :
//  - Host : la requete doit viser camera.local ou l'IP de la camera (bloque
//    l'attaque « DNS rebinding », ou un site se fait passer pour la camera) ;
//  - Origin : seules les pages du rover et de la camera peuvent lire les
//    reponses (avant : « * », n'importe quel site) ;
//  - actions (photo, reglages, vider la carte) : en-tete X-Rover obligatoire.
//    Un autre site ne peut pas l'envoyer (le navigateur demande d'abord
//    l'accord de la camera, refuse), ni par un simple lien ou une image.
// Le PC (IA, televersement) n'envoie pas d'Origin : il passe.
static const char* NOM_CAMERA = "camera.local";
static const char* NOM_ROVER_WEB = "rover.local";
// valeurs gardees en memoire : httpd envoie l'en-tete apres la fonction
static char origineRoverNom[32], origineRoverIp[32], origineCameraNom[32], origineCameraIp[32];

static bool memeHote(const char* valeur, const char* nom) {
  size_t n = strlen(nom);
  return strncasecmp(valeur, nom, n) == 0 && (valeur[n] == 0 || valeur[n] == ':' || valeur[n] == '/');
}

// origine autorisee -> la meme chaine, gardee en memoire ; sinon nullptr
static const char* origineConnue(const char* origine) {
  String ipRover = liaisonRoverIp().toString(), ipCam = WiFi.localIP().toString();
  snprintf(origineRoverNom, sizeof(origineRoverNom), "http://%s", NOM_ROVER_WEB);
  snprintf(origineRoverIp, sizeof(origineRoverIp), "http://%s", ipRover.c_str());
  snprintf(origineCameraNom, sizeof(origineCameraNom), "http://%s", NOM_CAMERA);
  snprintf(origineCameraIp, sizeof(origineCameraIp), "http://%s", ipCam.c_str());
  // l'IP du rover n'est connue qu'une fois la liaison etablie
  const char* connues[] = { origineRoverNom, origineCameraNom, origineCameraIp,
                            ipRover != "0.0.0.0" ? origineRoverIp : "" };
  for (const char* o : connues)
    if (*o && strcasecmp(origine, o) == 0) return o;
  return nullptr;
}

static esp_err_t refuser(httpd_req_t* req, const char* raison) {
  Serial.printf("[CAM] acces refuse (%s) : %s\n", raison, req->uri);
  httpd_resp_set_status(req, "403 Forbidden");
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_sendstr(req, "acces refuse");
  return ESP_OK;
}

// true : la requete peut continuer ; false : deja refusee (403)
static bool controleAcces(httpd_req_t* req, bool action) {
  char hote[48] = "", origine[64] = "", jeton[8] = "";
  httpd_req_get_hdr_value_str(req, "Host", hote, sizeof(hote));
  String ipCam = WiFi.localIP().toString();
  if (!memeHote(hote, NOM_CAMERA) && !memeHote(hote, ipCam.c_str())) {
    refuser(req, "adresse");
    return false;
  }
  if (httpd_req_get_hdr_value_str(req, "Origin", origine, sizeof(origine)) == ESP_OK) {
    const char* o = origineConnue(origine);
    if (!o) { refuser(req, "origine"); return false; }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", o);
    httpd_resp_set_hdr(req, "Vary", "Origin");
  }
  if (action && httpd_req_get_hdr_value_str(req, "X-Rover", jeton, sizeof(jeton)) != ESP_OK) {
    refuser(req, "action sans X-Rover");
    return false;
  }
  return true;
}

// pre-verification du navigateur avant une requete avec X-Rover
static esp_err_t preverification(httpd_req_t* req) {
  if (!controleAcces(req, false)) return ESP_OK;
  char origine[64] = "";
  if (httpd_req_get_hdr_value_str(req, "Origin", origine, sizeof(origine)) != ESP_OK)
    return refuser(req, "pre-verification sans origine");
  httpd_resp_set_status(req, "204 No Content");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "X-Rover");
  httpd_resp_set_hdr(req, "Access-Control-Max-Age", "600");
  return httpd_resp_send(req, nullptr, 0);
}

static esp_err_t envoyerJson(httpd_req_t* req, const char* json) {
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

// ---------- page de test ----------
static esp_err_t pageAccueil(httpd_req_t* req) {
  if (!controleAcces(req, false)) return ESP_OK;
  String ip = WiFi.localIP().toString();
  String page =
    "<!doctype html><html lang=fr><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>Camera rover</title><body style='font-family:system-ui;margin:16px'>"
    "<h1 style='font-size:20px'>Camera du rover</h1>"
    "<img src='http://" + ip + ":81/stream' style='max-width:100%;border-radius:8px;"
    "transform:rotate(" + String(cameraRotationActuelle()) + "deg)'>"
    "<p><a href='/capture'>Image</a> &middot; <a href='/status'>Etat</a></p>"
    "<p>Page complete : celle du rover (http://rover.local/).</p></body></html>";
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, page.c_str(), page.length());
}

// ---------- une image ----------
static esp_err_t capture(httpd_req_t* req) {
  if (!controleAcces(req, false)) return ESP_OK;
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  esp_err_t res = httpd_resp_send(req, (const char*) fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// ---------- photo sur la carte TF ----------
static esp_err_t photo(httpd_req_t* req) {
  if (!controleAcces(req, true)) return ESP_OK;
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
  if (!controleAcces(req, false)) return ESP_OK;
  char requete[96], chemin[64];
  if (httpd_req_get_url_query_str(req, requete, sizeof(requete)) != ESP_OK ||
      httpd_query_key_value(requete, "f", chemin, sizeof(chemin)) != ESP_OK ||
      (strncmp(chemin, "/photos/", 8) != 0 && strncmp(chemin, "/videos/", 8) != 0 &&
       strncmp(chemin, "/journal/", 9) != 0) ||
      strstr(chemin, "..")) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  File f = SD_MMC.open(chemin);
  if (!f) { httpd_resp_send_404(req); return ESP_FAIL; }
  // tout ce qui n'est pas une photo se telecharge (video AVI, mesures, journal)
  bool video = strncmp(chemin, "/photos/", 8) != 0;
  const char* ext = strrchr(chemin, '.');
  bool mesures = ext && !strcmp(ext, ".jsonl");
  httpd_resp_set_type(req, mesures ? "text/plain; charset=utf-8"
                         : video ? "video/x-msvideo" : "image/jpeg");
  if (video) {   // video et mesures se telechargent (le navigateur ne lit pas l'AVI)
    char dispo[64];
    snprintf(dispo, sizeof(dispo), "attachment; filename=%s", strrchr(chemin, '/') + 1);
    httpd_resp_set_hdr(req, "Content-Disposition", dispo);
  }
  // tampon hors de la pile (la tache du serveur n'a que quelques Ko)
  const size_t TAILLE_TAMPON = 8192;
  uint8_t* tampon = (uint8_t*) malloc(TAILLE_TAMPON);
  if (!tampon) { f.close(); httpd_resp_send_500(req); return ESP_FAIL; }
  size_t n;
  while ((n = f.read(tampon, TAILLE_TAMPON)) > 0) {
    if (httpd_resp_send_chunk(req, (const char*) tampon, n) != ESP_OK) break;
  }
  free(tampon);
  f.close();
  return httpd_resp_send_chunk(req, nullptr, 0);
}

// ---------- etat ----------
static esp_err_t repondreEtat(httpd_req_t* req) {
  char json[640];
  etatCameraJson(json, sizeof(json));
  return envoyerJson(req, json);
}

static esp_err_t status(httpd_req_t* req) {
  if (!controleAcces(req, false)) return ESP_OK;
  return repondreEtat(req);
}

// ---------- reglages ----------
static esp_err_t controle(httpd_req_t* req) {
  if (!controleAcces(req, true)) return ESP_OK;
  char requete[96], var[16], val[16];
  if (httpd_req_get_url_query_str(req, requete, sizeof(requete)) != ESP_OK ||
      httpd_query_key_value(requete, "var", var, sizeof(var)) != ESP_OK ||
      httpd_query_key_value(requete, "val", val, sizeof(val)) != ESP_OK) {
    return envoyerJson(req, "{\"ok\":0,\"erreur\":\"var et val attendus\"}");
  }
  bool ok = true;
  if (!strcmp(var, "res"))          ok = !enregistrementEnCours() && cameraResolution(val);
  else if (!strcmp(var, "qualite")) ok = cameraQualite(atoi(val));
  else if (!strcmp(var, "vflip"))   cameraRetourner(atoi(val), cameraMiroir());
  else if (!strcmp(var, "miroir"))  cameraRetourner(cameraRetourneeV(), atoi(val));
  else if (!strcmp(var, "rotation")) ok = cameraRotation(atoi(val));
  else if (!strcmp(var, "liaison")) liaisonRoverActiver(atoi(val));
  else if (!strcmp(var, "tx")) {           // puissance WiFi en dBm (2 a 19,5), pour essais ;
    float dbm = atof(val);                  // au demarrage : toujours le maximum
    ok = dbm >= 2 && dbm <= 19.5f;
    if (ok) WiFi.setTxPower((wifi_power_t) (int) (dbm * 4));
  }
  else if (!strcmp(var, "video")) {
    if (atoi(val)) ok = enregistrementDemarrer();
    else enregistrementArreter();
  }
  else ok = false;
  Serial.printf("[CAM] reglage %s = %s : %s\n", var, val, ok ? "ok" : "refuse");
  return repondreEtat(req);
}

// ---------- flux video MJPEG ----------
static esp_err_t flux(httpd_req_t* req) {
  if (!controleAcces(req, false)) return ESP_OK;
  esp_err_t res = httpd_resp_set_type(req, TYPE_FLUX);
  if (res != ESP_OK) return res;
  nbFlux++;
  Serial.println("[CAM] un ecran regarde la video");
  char entete[64];
  // L'image est copiee puis le tampon de la camera rendu aussitot : un ecran
  // lent (WiFi) ne bloque ainsi ni l'autre flux (page / IA) ni l'enregistrement.
  uint8_t* copie = nullptr;
  size_t tailleCopie = 0;
  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    size_t lgImage = fb->len;
    if (lgImage > tailleCopie) {
      free(copie);
      tailleCopie = lgImage + 16384;
      copie = (uint8_t*) ps_malloc(tailleCopie);
    }
    if (copie) memcpy(copie, fb->buf, lgImage);
    esp_camera_fb_return(fb);
    if (!copie) { res = ESP_ERR_NO_MEM; break; }
    size_t lg = snprintf(entete, sizeof(entete), ENTETE_IMAGE, (unsigned) lgImage);
    res = httpd_resp_send_chunk(req, SEPARATEUR, strlen(SEPARATEUR));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, entete, lg);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*) copie, lgImage);
    if (res != ESP_OK) break;     // ecran ferme
    cameraCompterImage();
  }
  free(copie);
  nbFlux--;
  Serial.println("[CAM] ecran video ferme");
  return res;
}

// ---------- journal de navigation ----------
static esp_err_t journaux(httpd_req_t* req) {
  if (!controleAcces(req, false)) return ESP_OK;
  String liste;
  journalListeJson(liste);
  String json = String("{\"courant\":\"") + journalFichierCourant() + "\",\"fichiers\":" + liste + "}";
  return envoyerJson(req, json.c_str());
}

// ---------- vider la carte TF : /vider?confirmer=oui ----------
int viderCarte() {
  if (enregistrementEnCours() || relaisMajEnCours()) return -1;
  int n = stockageViderPhotos();
  n += stockageViderDossier("/videos");
  enregistrementInit();                      // recompte : 0 video, numerotation a 1
  n += journalVider();
  Serial.printf("[SD] carte videe : %d fichier(s) efface(s)\n", n);
  return n;
}

static esp_err_t vider(httpd_req_t* req) {
  if (!controleAcces(req, true)) return ESP_OK;
  char requete[64], confirmer[8] = "";
  if (httpd_req_get_url_query_str(req, requete, sizeof(requete)) == ESP_OK)
    httpd_query_key_value(requete, "confirmer", confirmer, sizeof(confirmer));
  if (strcmp(confirmer, "oui") != 0)
    return envoyerJson(req, "{\"ok\":0,\"erreur\":\"ajouter ?confirmer=oui\"}");
  int n = viderCarte();
  if (n < 0) return envoyerJson(req, "{\"ok\":0,\"erreur\":\"enregistrement ou mise a jour en cours\"}");
  char json[64];
  snprintf(json, sizeof(json), "{\"ok\":1,\"effaces\":%d}", n);
  return envoyerJson(req, json);
}

// ---------- mise a jour du programme par HTTP (POST /maj) ----------
// Le PC envoie le firmware en POST brut, mot de passe dans X-Mot-De-Passe.
static esp_err_t miseAJour(httpd_req_t* req) {
  if (!controleAcces(req, false)) return ESP_OK;    // + mot de passe ci-dessous
  char mdp[64] = "";
  httpd_req_get_hdr_value_str(req, "X-Mot-De-Passe", mdp, sizeof(mdp));
  if (!motDePasseMaj || strcmp(mdp, motDePasseMaj) != 0) {
    httpd_resp_set_status(req, "403 Forbidden");
    return httpd_resp_sendstr(req, "mot de passe incorrect");
  }
  enregistrementArreter();
  Serial.printf("[MAJ] reception d'un nouveau programme (%u octets)...\n",
                (unsigned) req->content_len);
  if (!Update.begin(req->content_len)) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  const size_t TAILLE_TAMPON = 4096;
  uint8_t* tampon = (uint8_t*) malloc(TAILLE_TAMPON);
  if (!tampon) { Update.abort(); httpd_resp_send_500(req); return ESP_FAIL; }
  size_t reste = req->content_len;
  while (reste > 0) {
    int n = httpd_req_recv(req, (char*) tampon, min(reste, TAILLE_TAMPON));
    if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
    if (n <= 0 || Update.write(tampon, n) != (size_t) n) break;
    reste -= n;
  }
  free(tampon);
  if (reste > 0 || !Update.end(true)) {
    Update.abort();
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, (String("echec : ") + Update.errorString()).c_str());
  }
  httpd_resp_sendstr(req, "OK, redemarrage");
  Serial.println("[MAJ] OK, redemarrage");
  delay(500);
  ESP.restart();
  return ESP_OK;
}

void serveurCameraDemarrer(const char* motDePasse) {
  motDePasseMaj = motDePasse;
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = PORT_HTTP;
  cfg.ctrl_port = 32768;
  cfg.max_uri_handlers = 12;
  cfg.stack_size = 8192;
  cfg.uri_match_fn = httpd_uri_match_wildcard;   // "/*" : pre-verifications (OPTIONS)
  if (httpd_start(&serveurHttp, &cfg) == ESP_OK) {
    httpd_uri_t pages[] = {
      { "/",        HTTP_GET, pageAccueil, nullptr },
      { "/capture", HTTP_GET, capture,     nullptr },
      { "/photo",   HTTP_GET, photo,       nullptr },
      { "/sd",      HTTP_GET, lireFichier, nullptr },
      { "/status",  HTTP_GET, status,      nullptr },
      { "/control", HTTP_GET, controle,    nullptr },
      { "/maj",     HTTP_POST, miseAJour,  nullptr },
      { "/journaux", HTTP_GET, journaux,   nullptr },
      { "/vider",   HTTP_GET, vider,       nullptr },
      { "/*",       HTTP_OPTIONS, preverification, nullptr },
    };
    for (auto &p : pages) httpd_register_uri_handler(serveurHttp, &p);
  }

  // deux serveurs video : la page (81) et l'IA du PC (82)
  httpd_uri_t uriFlux = { "/stream", HTTP_GET, flux, nullptr };
  httpd_config_t cfgFlux = HTTPD_DEFAULT_CONFIG();
  cfgFlux.server_port = PORT_FLUX;
  cfgFlux.ctrl_port = 32769;
  cfgFlux.max_open_sockets = 3;            // 16 sockets en tout sur la carte
  if (httpd_start(&serveurFlux, &cfgFlux) == ESP_OK)
    httpd_register_uri_handler(serveurFlux, &uriFlux);
  cfgFlux.server_port = PORT_FLUX_IA;
  cfgFlux.ctrl_port = 32770;
  if (httpd_start(&serveurFluxIA, &cfgFlux) == ESP_OK)
    httpd_register_uri_handler(serveurFluxIA, &uriFlux);
  Serial.printf("[CAM] serveurs : http %u, video %u (page) et %u (IA)\n",
                PORT_HTTP, PORT_FLUX, PORT_FLUX_IA);
}
