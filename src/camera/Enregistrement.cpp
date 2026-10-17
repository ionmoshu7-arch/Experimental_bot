#include "Enregistrement.h"

#include <FS.h>
#include <SD_MMC.h>
#include "esp_camera.h"
#include "Stockage.h"

static const char* DOSSIER_VIDEOS = "/videos";
static const uint32_t LIBRE_MIN_MO = 20;          // arret si la carte est presque pleine
static const int IMAGES_MAX = DUREE_MAX_S * IMAGES_PAR_SECONDE_MAX + 100;

// ---------- format AVI ----------
// En-tete fixe de 224 octets, puis les images ("00dc"), puis l'index
// ("idx1"). Les tailles et le nombre d'images sont completes a la fin.
static const uint32_t TAILLE_ENTETE = 224;
static const uint32_t POS_TAILLE_RIFF = 4, POS_US_PAR_IMAGE = 32, POS_OCTETS_PAR_S = 36;
static const uint32_t POS_NB_IMAGES = 48, POS_LARGEUR = 64, POS_HAUTEUR = 68;
static const uint32_t POS_ECHELLE = 128, POS_CADENCE = 132, POS_LONGUEUR = 140;
static const uint32_t POS_RECT = 156, POS_STRF_LARGEUR = 176, POS_STRF_HAUTEUR = 180;
static const uint32_t POS_TAILLE_MOVI = 216, POS_MOVI = 220;

struct EntreeIndex { uint32_t position, taille; };

static TaskHandle_t tache = nullptr;
static volatile bool demandeArret = false, enCours = false;
static File fichier;
static File fichierMesures;
static SemaphoreHandle_t verrouMesures = nullptr;
static int nbMesures = 0;
static char nomFichier[32] = "";
static int prochainNumero = 1, nbVideos = 0;
static EntreeIndex* tableIndex = nullptr;
static int nbImages = 0;
static uint32_t positionEcriture = 0, tailleMax = 0;
static unsigned long debut = 0, duree = 0;

static void ecrire32(uint8_t* p, uint32_t v) {
  p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

static void ecrire16(uint8_t* p, uint16_t v) {
  p[0] = v; p[1] = v >> 8;
}

static void enteteAvi(uint8_t* h, uint16_t largeur, uint16_t hauteur) {
  memset(h, 0, TAILLE_ENTETE);
  memcpy(h + 0, "RIFF", 4);  memcpy(h + 8, "AVI ", 4);
  memcpy(h + 12, "LIST", 4); ecrire32(h + 16, 192); memcpy(h + 20, "hdrl", 4);
  memcpy(h + 24, "avih", 4); ecrire32(h + 28, 56);
  ecrire32(h + 44, 0x10);                         // AVIF_HASINDEX
  ecrire32(h + 56, 1);                            // un flux
  ecrire32(h + POS_LARGEUR, largeur); ecrire32(h + POS_HAUTEUR, hauteur);
  memcpy(h + 88, "LIST", 4); ecrire32(h + 92, 116); memcpy(h + 96, "strl", 4);
  memcpy(h + 100, "strh", 4); ecrire32(h + 104, 56);
  memcpy(h + 108, "vids", 4); memcpy(h + 112, "MJPG", 4);
  ecrire16(h + POS_RECT + 4, largeur); ecrire16(h + POS_RECT + 6, hauteur);
  memcpy(h + 164, "strf", 4); ecrire32(h + 168, 40);
  ecrire32(h + 172, 40);
  ecrire32(h + POS_STRF_LARGEUR, largeur); ecrire32(h + POS_STRF_HAUTEUR, hauteur);
  ecrire16(h + 184, 1); ecrire16(h + 186, 24);
  memcpy(h + 188, "MJPG", 4);
  ecrire32(h + 192, (uint32_t) largeur * hauteur * 3);
  memcpy(h + 212, "LIST", 4); memcpy(h + POS_MOVI, "movi", 4);
}

static void ecrireA(uint32_t position, uint32_t valeur) {
  uint8_t b[4];
  ecrire32(b, valeur);
  fichier.seek(position);
  fichier.write(b, 4);
}

// Index, tailles, cadence reelle : le fichier devient lisible
static void finaliser() {
  uint8_t b[16];
  memcpy(b, "idx1", 4);
  ecrire32(b + 4, nbImages * 16);
  fichier.seek(positionEcriture);
  fichier.write(b, 8);
  for (int i = 0; i < nbImages; i++) {
    memcpy(b, "00dc", 4);
    ecrire32(b + 4, 0x10);                        // image cle
    ecrire32(b + 8, tableIndex[i].position);
    ecrire32(b + 12, tableIndex[i].taille);
    fichier.write(b, 16);
  }
  uint32_t tailleFichier = positionEcriture + 8 + nbImages * 16;

  float ips = duree > 0 ? nbImages * 1000.0f / duree : IMAGES_PAR_SECONDE_MAX;
  if (ips < 0.1f) ips = 0.1f;
  ecrireA(POS_TAILLE_RIFF, tailleFichier - 8);
  ecrireA(POS_US_PAR_IMAGE, (uint32_t) (1000000.0f / ips));
  ecrireA(POS_OCTETS_PAR_S, (uint32_t) (tailleMax * ips));
  ecrireA(POS_NB_IMAGES, nbImages);
  ecrireA(POS_ECHELLE, 1000);
  ecrireA(POS_CADENCE, (uint32_t) (ips * 1000));
  ecrireA(POS_LONGUEUR, nbImages);
  ecrireA(POS_TAILLE_MOVI, positionEcriture - POS_MOVI);
  fichier.close();

  xSemaphoreTake(verrouMesures, portMAX_DELAY);
  if (fichierMesures) fichierMesures.close();
  xSemaphoreGive(verrouMesures);
}

static void tacheEnregistrement(void*) {
  const unsigned long periode = 1000 / IMAGES_PAR_SECONDE_MAX;
  const char* raison = "demande";
  unsigned long prochaine = millis();

  while (!demandeArret) {
    if (millis() - debut >= (unsigned long) DUREE_MAX_S * 1000) { raison = "duree maximale"; break; }
    if (nbImages >= IMAGES_MAX) { raison = "duree maximale"; break; }
    if (nbImages % 50 == 0 && stockageLibreMo() < LIBRE_MIN_MO) { raison = "carte presque pleine"; break; }

    long attente = (long) (prochaine - millis());
    if (attente > 0) vTaskDelay(pdMS_TO_TICKS(attente));
    prochaine += periode;

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) continue;
    uint8_t entete[8];
    memcpy(entete, "00dc", 4);
    ecrire32(entete + 4, fb->len);
    bool ok = fichier.write(entete, 8) == 8 && fichier.write(fb->buf, fb->len) == fb->len;
    uint32_t taille = fb->len;
    esp_camera_fb_return(fb);
    if (taille & 1) ok = ok && fichier.write((const uint8_t*) "", 1) == 1;   // alignement pair
    if (!ok) { raison = "erreur d'ecriture"; break; }

    tableIndex[nbImages++] = { positionEcriture - POS_MOVI, taille };
    positionEcriture += 8 + taille + (taille & 1);
    if (taille > tailleMax) tailleMax = taille;
  }

  duree = millis() - debut;
  finaliser();
  nbVideos++;
  Serial.printf("[VIDEO] %s termine (%s) : %d images, %.1f s, %d mesures du rover\n",
                nomFichier, raison, nbImages, duree / 1000.0, nbMesures);
  enCours = false;
  tache = nullptr;
  vTaskDelete(nullptr);
}

void enregistrementInit() {
  if (!stockagePresent()) return;
  SD_MMC.mkdir(DOSSIER_VIDEOS);
  File dossier = SD_MMC.open(DOSSIER_VIDEOS);
  for (File f = dossier.openNextFile(); f; f = dossier.openNextFile()) {
    int n = 0;
    if (sscanf(f.name(), "VID_%d.avi", &n) == 1) {
      nbVideos++;
      if (n >= prochainNumero) prochainNumero = n + 1;
    }
  }
  if (!verrouMesures) verrouMesures = xSemaphoreCreateMutex();
  if (!tableIndex) tableIndex = (EntreeIndex*) ps_malloc(IMAGES_MAX * sizeof(EntreeIndex));
  Serial.printf("[VIDEO] %d video(s) sur la carte\n", nbVideos);
}

bool enregistrementDemarrer() {
  if (enCours || !stockagePresent() || !tableIndex) return false;
  if (stockageLibreMo() < LIBRE_MIN_MO) return false;

  camera_fb_t* fb = esp_camera_fb_get();          // dimensions de l'image actuelle
  if (!fb) return false;
  uint16_t largeur = fb->width, hauteur = fb->height;
  esp_camera_fb_return(fb);

  snprintf(nomFichier, sizeof(nomFichier), "%s/VID_%04d.avi", DOSSIER_VIDEOS, prochainNumero);
  fichier = SD_MMC.open(nomFichier, FILE_WRITE);
  if (!fichier) return false;
  uint8_t entete[TAILLE_ENTETE];
  enteteAvi(entete, largeur, hauteur);
  if (fichier.write(entete, TAILLE_ENTETE) != TAILLE_ENTETE) {
    fichier.close();
    return false;
  }
  prochainNumero++;

  // mesures du rover, meme nom que la video en .jsonl
  char nomMesures[32];
  strncpy(nomMesures, nomFichier, sizeof(nomMesures));
  char* ext = strrchr(nomMesures, '.');
  if (ext) strcpy(ext, ".jsonl");
  xSemaphoreTake(verrouMesures, portMAX_DELAY);
  fichierMesures = SD_MMC.open(nomMesures, FILE_WRITE);
  nbMesures = 0;
  xSemaphoreGive(verrouMesures);

  positionEcriture = TAILLE_ENTETE;
  nbImages = 0;
  tailleMax = 0;
  duree = 0;
  debut = millis();
  demandeArret = false;
  enCours = true;
  xTaskCreatePinnedToCore(tacheEnregistrement, "video", 6144, nullptr, 2, &tache, 0);
  Serial.printf("[VIDEO] enregistrement %s (%ux%u)\n", nomFichier, largeur, hauteur);
  return true;
}

void enregistrementArreter() {
  if (enCours) demandeArret = true;
}

bool enregistrementEnCours() { return enCours; }

int enregistrementSecondes() {
  return enCours ? (millis() - debut) / 1000 : duree / 1000;
}

const char* enregistrementFichier() { return nomFichier; }

void enregistrementMesure(const uint8_t* json, size_t longueur) {
  if (!enCours || !verrouMesures) return;
  xSemaphoreTake(verrouMesures, portMAX_DELAY);
  if (fichierMesures) {
    char temps[16];
    int n = snprintf(temps, sizeof(temps), "%lu\t", millis() - debut);
    fichierMesures.write((const uint8_t*) temps, n);
    fichierMesures.write(json, longueur);
    fichierMesures.write((const uint8_t*) "\n", 1);
    nbMesures++;
  }
  xSemaphoreGive(verrouMesures);
}
int enregistrementNbVideos() { return nbVideos; }
