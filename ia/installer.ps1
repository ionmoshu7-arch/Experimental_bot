# Installe l'environnement Python de l'IA du rover.
# L'environnement (~1 Go avec PyTorch) et le modele YOLO vont dans
# %LOCALAPPDATA%\RoverIA, HORS du dossier OneDrive du projet.
#   powershell -ExecutionPolicy Bypass -File ia\installer.ps1

$python = "$env:LOCALAPPDATA\Programs\Python\Python312\python.exe"
if (-not (Test-Path $python)) {
    Write-Host "Python 3.12 absent : installation (compte utilisateur, sans droits administrateur)..."
    winget install --id Python.Python.3.12 -e --scope user --silent --accept-package-agreements --accept-source-agreements
}
$venv = "$env:LOCALAPPDATA\RoverIA\venv"
if (-not (Test-Path "$venv\Scripts\python.exe")) { & $python -m venv $venv }
& "$venv\Scripts\python.exe" -m pip install --upgrade pip
# --upgrade : relancer ce script met aussi a jour YOLO, PyTorch et OpenCV
& "$venv\Scripts\python.exe" -m pip install --upgrade -r "$PSScriptRoot\requirements.txt"
Write-Host "Pret. Lancer depuis VS Code : Terminal > Executer la tache > IA rover : voir / suivre"
