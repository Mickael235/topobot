# 00 — Démarrage rapide

Objectif : faire **bouger le robot** et **lancer une mission** le plus vite
possible. Les détails sont dans les chapitres suivants ; ici on va à
l'essentiel.

> ⚠️ **Sécurité d'abord.** La première fois, **surélevez le robot** (roues
> dans le vide) ou placez‑le dans une zone dégagée d'au moins 2 m. Gardez la
> main près du **bouton d'arrêt d'urgence (ARU)** et de l'alimentation
> puissance. Les moteurs peuvent démarrer brutalement.

> 🔴 **Réflexe à prendre dès le départ.** Avant **chaque** session de test,
> lancez `python test_encoders.py <PORT>` et **tournez les 4 roues à la main**
> pour vérifier que les tics sont bien comptés. Les 4 encodeurs marchent, mais
> la **carte de commande** est intermittente : une roue qui ne compte pas
> fausse toute l'odométrie. Voir [chapitre 5](05_Tests/README.md).

---

## 1. Ce qu'il vous faut

- Le robot assemblé, batterie chargée.
- Un **STM32 F446RE** flashé avec le firmware (voir étape 2).
- Une **Raspberry Pi** reliée au STM32 par USB (câble ST‑Link) et au XBee.
- Un PC ou une tablette sur le même réseau que la Pi.
- Logiciels : [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
  (firmware), Python 3.11+ et Node.js (application Pi).

---

## 2. Flasher le firmware STM32 (5 min)

1. Ouvrir **STM32CubeIDE** → *File ▸ Open Projects from File System…* → choisir
   le dossier `ProjetRobot`.
2. Brancher la carte en USB (ST‑Link intégré sur la Nucleo).
3. Cliquer sur le marteau 🔨 (*Build*), puis sur l'insecte vert 🐞 (*Debug*) ou
   *Run*. Le binaire est flashé automatiquement.

Détails et dépannage compilation : [chapitre 3](03_Firmware_STM32/README.md).

---

## 3. Tester les moteurs sans la Pi (optionnel mais recommandé)

Branchez votre PC directement sur le STM32 (port série virtuel ST‑Link) et
lancez, depuis le dossier `ProjetRobot` :

```bash
python test_comm.py COM6 --vx 0.08 --duration 2
```

(Remplacez `COM6` par votre port — sous Linux/Pi : `/dev/serial0` ou
`/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_...`.)

Le robot doit avancer, reculer, puis tourner. Si rien ne bouge → voir
[dépannage](06_Calibration_Depannage.md).

---

## 4. Lancer l'application TopoBot sur la Raspberry Pi (10 min)

### Backend (serveur)

```bash
cd topobot/topobot/backend
python3 -m venv venv            # si pas déjà fait
source venv/bin/activate
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port 8000
```

Le serveur démarre. Vérifiez en bas du log qu'il a trouvé `frontend/dist`
(sinon : `cd ../frontend && npm install && npm run build`).

### Interface web

Depuis un navigateur sur le réseau : **`http://<IP_de_la_Pi>:8000`**

1. Écran de connexion → *Connecter*.
2. Écran de configuration → saisir largeur X, longueur Y et le pas de mesure.
3. *Démarrer la mission*. Le robot parcourt la grille, mesure à chaque point,
   et l'interface affiche la progression en temps réel.

Le gros **bouton rouge d'arrêt d'urgence** de l'interface coupe tout
immédiatement (mission + moteurs + tracking station).

Détails : [backend](04_Raspberry_Pi/README.md) ·
[interface](04_Raspberry_Pi/Interface_Web.md) ·
[station Topcon](04_Raspberry_Pi/Station_Topcon.md).

---

## 5. Vérifier que la station Topcon répond (avant une vraie mission)

```bash
python ms1ax.py /dev/ttyUSB0 mes      # lance une mesure simple
```

Si la station renvoie une trame CSV avec des nombres → la liaison XBee marche.
Sinon → [Station Topcon](04_Raspberry_Pi/Station_Topcon.md) et
[dépannage](06_Calibration_Depannage.md).

---

## En cas de problème

1. Le robot ne répond plus du tout / réception bloquée → **reset du STM32**
   (bouton noir sur la Nucleo). Voir le bug d'overrun UART décrit dans la
   [passation](08_Passation/README.md).
2. Une seule roue tourne, le robot part de travers → problème de seuil moteur,
   voir [calibration](06_Calibration_Depannage.md).
3. La station ne mesure pas → vérifier le port `/dev/ttyUSB0`, le XBee, et que
   la station vise bien le prisme.

> Pour tout comprendre en profondeur, continuez avec la
> [vue d'ensemble](01_Vue_Ensemble/README.md).
