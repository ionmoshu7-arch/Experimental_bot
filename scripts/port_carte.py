# Choisit le port serie d'une carte d'apres sa puce USB.
#
# Windows peut donner n'importe quel numero de COM au rover (puce CH343,
# numero de serie unique) et a la camera (puce CH340, sans numero de serie).
# Sans ce script, PlatformIO pourrait televerser le programme du rover dans
# la camera, ou l'inverse.
#
# Options dans platformio.ini (une seule suffit) :
#   custom_usb_serie   = numero de serie USB de la carte visee
#   custom_usb_vidpid  = type de puce USB "VID:PID" de la carte visee, pour
#                        une puce sans numero de serie (CH340 : 1A86:7523)
#   custom_usb_exclure = numero(s) de serie a ne jamais utiliser
# Les valeurs s'affichent avec : pio device list   (VID:PID=... SER=...)
# Sans effet pour un televersement par WiFi (OTA).

Import("env")

import sys
from serial.tools import list_ports

if env.GetProjectOption("upload_protocol", "esptool") == "esptool":
    serie = env.GetProjectOption("custom_usb_serie", "").strip()
    vidpid = env.GetProjectOption("custom_usb_vidpid", "").strip().upper()
    exclus = [s.strip() for s in env.GetProjectOption("custom_usb_exclure", "").split(",") if s.strip()]
    ports = list(list_ports.comports())
    nom_env = env["PIOENV"]

    if serie:
        trouves = [p.device for p in ports if (p.serial_number or "") == serie]
    elif vidpid:
        trouves = [p.device for p in ports
                   if p.vid is not None and "%04X:%04X" % (p.vid, p.pid) == vidpid]
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
                % (nom_env, serie or vidpid or "autre que " + ", ".join(exclus)))
            env.Exit(1)
