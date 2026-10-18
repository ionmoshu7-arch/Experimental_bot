#include "Pont.h"

#include "esp_camera.h"
#include "Camera.h"
#include "Enregistrement.h"
#include "Journal.h"
#include "LiaisonRover.h"
#include "RelaisMaj.h"
#include "ServeurCamera.h"
#include "Stockage.h"

static const unsigned long PERIODE_RESUME_MS = 2000;

static char ligne[96];
static uint8_t lg = 0;
static unsigned long dernierResume = 0;

void pontDemarrer() {
  Serial.println("[PONT] console des 2 cartes : lignes -> rover, \"cam ...\" -> camera, \"cam aide\"");
}

// ---------- rover -> cable ----------

// Valeur numerique qui suit "cle" dans le texte JSON, a partir de "depuis"
static const char* chercher(const char* texte, const char* cle, float &valeur) {
  const char* p = strstr(texte, cle);
  if (!p) return nullptr;
  valeur = atof(p + strlen(cle));
  return p + strlen(cle);
}

static void texteChamp(const char* texte, const char* cle, char* dest, size_t taille) {
  const char* p = strstr(texte, cle);
  dest[0] = 0;
  if (!p) return;
  p += strlen(cle);
  size_t i = 0;
  while (*p && *p != '"' && i < taille - 1) dest[i++] = *p++;
  dest[i] = 0;
}

static void resumerData(const char* json) {
  float x = 0, y = 0, cap = 0, col = 0;
  chercher(json, "\"x\":", x);
  chercher(json, "\"y\":", y);
  const char* pose = strstr(json, "\"pose\":");
  if (pose) chercher(pose, "\"cap\":", cap);

  // distances des 4 capteurs, dans l'ordre avant, arriere, droite, gauche
  float d[4] = { -1, -1, -1, -1 };
  const char* p = strstr(json, "\"tof\":");
  for (int i = 0; i < 4 && p; i++) p = chercher(p, "\"d\":", d[i]);

  char ligneEtat[16], nav[20];
  const char* pl = strstr(json, "\"ligne\":");
  const char* pn = strstr(json, "\"nav\":");
  if (pl) texteChamp(pl, "\"e\":\"", ligneEtat, sizeof(ligneEtat)); else strcpy(ligneEtat, "?");
  if (pn) { texteChamp(pn, "\"e\":\"", nav, sizeof(nav)); chercher(pn, "\"col\":", col); }
  else strcpy(nav, "?");

  Serial.printf("[ROVER] x=%.2f y=%.2f m cap=%.0f | av=%.0f ar=%.0f dr=%.0f ga=%.0f mm"
                " | ligne=%s nav=%s collisions=%.0f\n",
                x / 1000, y / 1000, cap, d[0], d[1], d[2], d[3], ligneEtat, nav, col);
}

void pontMessageRover(const uint8_t* texte, size_t longueur) {
  if (relaisMajEnCours()) return;                  // le cable sert au programme
  if (longueur > 6 && !memcmp(texte, "[DATA]", 6)) {
    if (millis() - dernierResume < PERIODE_RESUME_MS) return;
    dernierResume = millis();
    char* copie = (char*) malloc(longueur + 1);
    if (!copie) return;
    memcpy(copie, texte, longueur);
    copie[longueur] = 0;
    resumerData(copie);
    free(copie);
    return;
  }
  Serial.print("[ROVER] ");
  Serial.write(texte, longueur);
  Serial.println();
}

// ---------- commandes de la camera ----------

void pontCommandeCamera(const char* c) {
  if (!strcmp(c, "photo")) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { Serial.println("[CAM] pas d'image"); return; }
    String chemin = stockageEnregistrerPhoto(fb->buf, fb->len);
    esp_camera_fb_return(fb);
    Serial.println(chemin.isEmpty() ? "[CAM] photo impossible (carte TF ?)" : "[CAM] photo " + chemin);
  } else if (!strncmp(c, "video ", 6)) {
    if (atoi(c + 6)) Serial.println(enregistrementDemarrer() ? "[CAM] enregistrement demarre"
                                                             : "[CAM] enregistrement impossible");
    else { enregistrementArreter(); Serial.println("[CAM] enregistrement arrete"); }
  } else if (!strncmp(c, "res ", 4)) {
    Serial.println(cameraResolution(c + 4) ? "[CAM] resolution changee" : "[CAM] resolution refusee");
  } else if (!strncmp(c, "liaison ", 8)) {
    liaisonRoverActiver(atoi(c + 8));
  } else if (!strncmp(c, "vider", 5)) {
    if (strcmp(c, "vider oui") != 0) { Serial.println("[CAM] pour tout effacer : cam vider oui"); return; }
    int n = viderCarte();
    if (n < 0) Serial.println("[CAM] impossible : enregistrement ou mise a jour en cours");
    else Serial.printf("[CAM] carte videe : %d fichier(s) efface(s)\n", n);
  } else if (!strcmp(c, "journal")) {
    String liste;
    journalListeJson(liste);
    Serial.printf("[CAM] journal en cours : %s\n[CAM] fichiers : %s\n", journalFichierCourant(), liste.c_str());
  } else if (!strcmp(c, "etat")) {
    char json[640];
    etatCameraJson(json, sizeof(json));
    Serial.print("[CAM] ");
    Serial.println(json);
  } else {
    Serial.println("[CAM] commandes : cam photo | cam video 1/0 | cam res QVGA..UXGA | "
                   "cam etat | cam liaison 1/0 | cam journal | cam vider oui");
    Serial.println("[PONT] autres lignes -> rover : s stop, c calibrer, z cap 0, O origine, "
                   "T pente 0, L 1/0 ligne, N 1/0 navigation");
  }
}

static void executerLigne() {
  ligne[lg] = 0;
  if (lg == 0) return;
  if (!strncmp(ligne, "!MAJROVER", 9)) {
    relaisMajRecevoir(ligne + 9);                  // prend la main sur le cable
  } else if (!strcmp(ligne, "!PING")) {
    Serial.println("[PONT] PONG");
  } else if (!strncmp(ligne, "cam ", 4) || !strcmp(ligne, "cam")) {
    pontCommandeCamera(lg > 4 ? ligne + 4 : "aide");
  } else {
    char message[sizeof(ligne) + 2];
    snprintf(message, sizeof(message), "%s\n", ligne);
    if (!liaisonRoverEnvoyer(message)) Serial.println("[PONT] rover non connecte");
  }
}

void pontMettreAJour() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      executerLigne();
      lg = 0;
    } else if (lg < sizeof(ligne) - 1) {
      ligne[lg++] = ch;
    }
  }
}
