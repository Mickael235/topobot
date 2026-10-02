# 4 — Raspberry Pi / Application TopoBot

La Raspberry Pi héberge l'application **TopoBot Manager** : un serveur Python
qui orchestre la mission, pilote le STM32 et la station Topcon, et sert
l'interface web.

Code : `topobot/topobot/`
- `backend/` — serveur FastAPI (Python)
- `frontend/` — interface web (React + Vite)

Voir aussi :
- [Interface Web](Interface_Web.md) — le frontend React
- [Station Topcon MS1AX](Station_Topcon.md) — la mesure topographique

## 4.1 Installation

### Backend (Python)

```bash
cd topobot/topobot/backend
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Dépendances (`requirements.txt`) : FastAPI, Uvicorn, SQLAlchemy, requests,
python‑dotenv, **pyserial**.

### Frontend (Node)

```bash
cd topobot/topobot/frontend
npm install
npm run build        # produit frontend/dist/ servi par le backend
```

> Le backend sert automatiquement `frontend/dist/`. Tant que ce dossier n'existe
> pas, le serveur affiche `[WARN] frontend/dist inexistant — lancer npm run build`.

## 4.2 Configuration des ports série

Dans `backend/main.py`, en haut du fichier :

```python
STM32_PORT  = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_...-if02"
STM32_BAUD  = 115200
TOPCON_PORT = "/dev/ttyUSB0"      # XBee via adaptateur FTDI
TOPCON_BAUD = 9600
```

> ⚠️ **À vérifier sur chaque Pi.** Les chemins peuvent changer. Listez les
> ports avec `ls /dev/serial/by-id/` (stable) ou `dmesg | grep tty`. Préférez
> toujours `/dev/serial/by-id/...` qui ne bouge pas selon l'ordre de branchement.

## 4.3 Lancement

```bash
cd topobot/topobot/backend
source venv/bin/activate
uvicorn main:app --host 0.0.0.0 --port 8000
```

Puis depuis un navigateur du réseau : `http://<IP_de_la_Pi>:8000`.

> Pour un lancement automatique au démarrage de la Pi, créez un service
> `systemd` (piste d'amélioration, voir [passation](../08_Passation/README.md)).

## 4.4 Ce que fait le backend

### Génération de la grille (boustrophédon)

`generate_waypoints(x_max, y_max, step)` produit la liste des points en
serpentin. Ligne paire : x croissant ; ligne impaire : x décroissant. Le point
(0,0) est inclus en premier (le robot y est déjà au départ).

### Routes REST

| Route | Méthode | Rôle |
|-------|---------|------|
| `/start-mission` | POST | Enregistre la config (X, Y, pas) et vide la base. |
| `/stop` | POST | **Arrêt d'urgence** : abort mission + `CMD_STOP` + coupe tracking. |
| `/status` | GET | Renvoie pose + état du robot (`GET_STATUS`). |
| `/results` | GET | Renvoie tous les points mesurés (base SQLite). |
| `/topcon/std` | POST | Station en mode standard. |
| `/topcon/mes` | POST | Lance une mesure simple. |
| `/topcon/track` | POST | Active le mode tracking. |
| `/topcon/track/stop` | POST | Arrête le tracking. |

### WebSocket `/ws` — la mission

C'est le **chef d'orchestre**. Pour chaque waypoint : déplacement, arrêt
tracking, déploiement prisme, mesure précise, repli prisme, sauvegarde, reprise
tracking. Diffuse en temps réel les logs, la position et la progression au
frontend. Surveille en permanence le flag d'abort (bouton d'arrêt d'urgence).
Voir le déroulé détaillé dans [Vue d'ensemble §1.4](../01_Vue_Ensemble/README.md#14-déroulé-détaillé-dune-mission-côté-logiciel).

### Communication avec le STM32

Le backend ré‑implémente le [protocole binaire](../03_Firmware_STM32/Protocole_Communication.md)
en Python : `_build_frame`, `_read_frame`, `_crc`, puis les fonctions de haut
niveau `_stm32_move_to`, `_stm32_send_cmd`, `_stm32_get_status`. Un
**verrou** (`stm32_lock`) protège l'accès série, **sauf** l'arrêt d'urgence qui
écrit directement pour ne pas attendre la fin d'un `MOVE_TO` en cours.

## 4.5 Base de données

SQLite, fichier `backend/topobot.db`, table `points` :

| Colonne | Description |
|---------|-------------|
| id | clé primaire |
| time | heure de la mesure (HH:MM:SS) |
| x, y | coordonnées du point dans la grille (m) |
| hz | angle horizontal mesuré (gon) |
| v | angle vertical mesuré (gon) |
| dist | distance station→prisme (m) |

> Le **Z (altitude)** n'est pas stocké : la station ne fournit que Hz, V et la
> distance. Le Z se recalcule ensuite par trigonométrie (piste à développer,
> voir [passation](../08_Passation/README.md)).

## 4.6 Fichiers `main*.py` : lequel utiliser ?

Le dossier `backend/` contient plusieurs variantes (héritage de
l'expérimentation) :

| Fichier | Statut |
|---------|--------|
| `main.py` | ✅ **Version de référence** (avec station Topcon, tracking). |
| `mainSansTop.py` | Variante **sans station Topcon** (test des déplacements seuls). |
| `main_old.py`, `main.py.save`, `main.txt`, `main2.txt` | Anciennes versions / sauvegardes — à archiver ou supprimer. |

> Recommandation : conservez `main.py` (et éventuellement `mainSansTop.py` pour
> les tests sans station), archivez le reste pour éviter la confusion.
