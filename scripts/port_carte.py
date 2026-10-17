# Choisit le port serie d'une carte d'apres le numero de serie de sa puce USB.
#
# Le rover et la camera utilisent la meme puce (CH343) : Windows peut leur
# donner n'importe quel numero de COM. Sans ce script, PlatformIO pourrait
# televerser le programme du rover dans la camera, ou l'inverse.
#
# Options dans platformio.ini :
#   custom_usb_serie   = numero de serie de la carte visee (obligatoire)
#   custom_usb_exclure = numero(s) de serie a ne jamais utiliser, separes par
#                        des virgules (quand celui de la carte n'est pas connu)
# Les numeros s'affichent avec : pio device list   (champ SER=...)
# Sans effet pour un televersement par WiFi (OTA).

Import("env")

import sys
from serial.tools import list_ports

if env.GetProjectOption("upload_protocol", "esptool") == "esptool":
    serie = env.GetProjectOption("custom_usb_serie", "").strip()
    exclus = [s.strip() for s in env.GetProjectOption("custom_usb_exclure", "").split(",") if s.strip()]
    ports = list(list_ports.comports())
    nom_env = env["PIOENV"]

    if serie:
        trouves = [p.device for p in ports if (p.serial_number or "") == serie]
    elif exclus:
        trouves = [p.device for p in ports
                   if p.serial_number and p.serial_number not in exclus and p.vid]
    else:
        trouves = None   # rien de configure : detection automatique

    if trouves is not None:
        if trouves:
            env.Replace(UPLOAD_PORT=trouves[0])
            print("[%s] carte trouvee sur %s" % (nom_env, trouves[0]))
        elif "upload" in COMMAND_LINE_TARGETS:
            sys.stderr.write(
                "\n[%s] carte introuvable (numero de serie USB %s). "
                "Est-elle branchee ? Voir : pio device list\n\n"
                % (nom_env, serie or "autre que " + ", ".join(exclus)))
            env.Exit(1)
