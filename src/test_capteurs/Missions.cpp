#include "Missions.h"

#include "Navigation.h"
#include "Odometrie.h"
#include "Reseau.h"
#include "Telemetrie.h"

static const unsigned long PAUSE_POINT_MS = 1500;   // arret a chaque point de patrouille

struct Point { float x, y; };

static InfoMission info = { MI_AUCUNE, "", 0, 0, 0, false, "" };
static Point points[MAX_POINTS_PATROUILLE];
static unsigned long finExploration = 0, debutPause = 0;
static bool enPause = false, retourOriente = false;

const InfoMission& missionInfo() { return info; }
bool missionActive() { return info.mission != MI_AUCUNE; }
bool missionPiloteeParIA() { return info.mission == MI_SUIVRE; }

const char* missionNom() {
  switch (info.mission) {
    case MI_EXPLORER:   return "explorer";
    case MI_BASE:       return "base";
    case MI_PATROUILLE: return "patrouille";
    case MI_MUR:        return "mur";
    case MI_SUIVRE:     return "suivre";
    default:            return "";
  }
}

void missionArreter(const char* raison) {
  if (info.mission == MI_AUCUNE) return;
  if (info.mission == MI_SUIVRE) reseauEnvoyerIA("[POURIA] stop");
  info.mission = MI_AUCUNE;
  info.etat = "";
  navigationLibre();
  navigationArreter(nullptr);
  if (raison) journal("[MISSION] %s", raison);
}

static void demarrer(Mission m, const char* etat) {
  missionArreter(nullptr);
  info.mission = m;
  info.etat = etat;
  enPause = retourOriente = false;
}

static void allerAuPoint(int i) {
  info.point = i;
  info.etat = "vers un point";
  navigationAllerVers(points[i].x, points[i].y);
  journal("[MISSION] patrouille : point %d/%d (%.2f, %.2f) m", i + 1, info.nbPoints,
          points[i].x / 1000, points[i].y / 1000);
}

void missionCommande(const char* args) {
  char mot[16] = "", param[24] = "";
  sscanf(args, " %15s %23s", mot, param);

  if (!strcmp(mot, "stop")) {
    missionArreter("arretee");
  } else if (!strcmp(mot, "explorer")) {
    int minutes = atoi(param);
    if (minutes <= 0) minutes = 5;
    demarrer(MI_EXPLORER, "exploration");
    finExploration = millis() + minutes * 60000UL;
    navigationLibre();
    navigationDemarrer();
    if (!navigationActive()) { missionArreter("exploration impossible (capteurs ?)"); return; }
    journal("[MISSION] explorer %d min", minutes);
  } else if (!strcmp(mot, "base")) {
    demarrer(MI_BASE, "retour a la base");
    navigationAllerVers(0, 0);
    if (!navigationActive()) { missionArreter("retour impossible (capteurs ?)"); return; }
    journal("[MISSION] retour a la base");
  } else if (!strcmp(mot, "point")) {
    if (info.nbPoints >= MAX_POINTS_PATROUILLE) { journal("[MISSION] deja %d points", MAX_POINTS_PATROUILLE); return; }
    const Pose &p = odometriePose();
    points[info.nbPoints++] = { p.x, p.y };
    journal("[MISSION] point %d memorise (%.2f, %.2f) m", info.nbPoints, p.x / 1000, p.y / 1000);
  } else if (!strcmp(mot, "effacer")) {
    if (info.mission == MI_PATROUILLE) missionArreter("patrouille arretee");
    info.nbPoints = 0;
    journal("[MISSION] points de patrouille effaces");
  } else if (!strcmp(mot, "photos")) {
    info.photos = atoi(param) != 0;
    journal("[MISSION] photo a chaque point : %s", info.photos ? "oui" : "non");
  } else if (!strcmp(mot, "patrouille")) {
    if (info.nbPoints < 2) { journal("[MISSION] patrouille : memoriser au moins 2 points"); return; }
    demarrer(MI_PATROUILLE, "patrouille");
    allerAuPoint(0);
    if (!navigationActive()) missionArreter("patrouille impossible (capteurs ?)");
  } else if (!strcmp(mot, "mur")) {
    bool gauche = param[0] != 'd';
    demarrer(MI_MUR, gauche ? "longe le mur gauche" : "longe le mur droit");
    navigationLongerMur(gauche ? 1 : -1);
    if (!navigationActive()) missionArreter("mur impossible (capteurs ?)");
  } else if (!strcmp(mot, "suivre")) {
    if (!reseauIAConnectee()) {
      journal("[MISSION] suivre : l'IA du PC n'est pas lancee (VS Code > tache IA rover)");
      return;
    }
    demarrer(MI_SUIVRE, "suivi par l'IA");
    strncpy(info.cible, param[0] ? param : "person", sizeof(info.cible) - 1);
    char ordre[48];
    snprintf(ordre, sizeof(ordre), "[POURIA] suivre %s", info.cible);
    reseauEnvoyerIA(ordre);
    journal("[MISSION] suivre : %s (IA du PC)", info.cible);
  } else {
    journal("[MISSION] inconnue : %s", args);
  }
}

void missionsMettreAJour() {
  switch (info.mission) {
    case MI_EXPLORER: {
      long reste = (long) (finExploration - millis());
      info.resteS = reste > 0 ? reste / 1000 : 0;
      if (reste <= 0) missionArreter("exploration terminee");
      else if (!navigationActive()) missionArreter("exploration interrompue");
      break;
    }
    case MI_BASE:
      if (navigationButAtteint()) {
        if (!retourOriente) {               // arrive : se tourner vers le cap de depart
          retourOriente = true;
          info.etat = "orientation";
          navigationOrienter(0);
        } else {
          missionArreter("rover revenu a la base");
        }
      } else if (!navigationActive()) {
        missionArreter("retour interrompu");
      }
      break;
    case MI_PATROUILLE:
      if (enPause) {
        if (millis() - debutPause >= PAUSE_POINT_MS) {
          enPause = false;
          allerAuPoint((info.point + 1) % info.nbPoints);
        }
      } else if (navigationButAtteint()) {
        enPause = true;
        debutPause = millis();
        info.etat = "au point";
        if (info.photos) reseauEnvoyerCamera("[POURCAM] photo");
      } else if (!navigationActive()) {
        missionArreter("patrouille interrompue");
      }
      break;
    case MI_MUR:
      if (!navigationActive()) missionArreter("tour du mur interrompu");
      break;
    default:
      break;
  }
}
