# 5 — Catalogue des scripts de test

Tous les scripts `test_*.py` sont à la **racine** du projet. Ils ont servi à
valider chaque brique, dans l'ordre, et restent les meilleurs outils de
diagnostic. Ce chapitre explique **quoi lancer, quand, et dans quel ordre**.

> ⚠️ **Sécurité** : pour tout test qui commande les moteurs, **surélevez le
> robot** (roues dans le vide) la première fois, ou placez‑le dans une zone
> dégagée. Gardez le bouton ARU et l'alimentation à portée de main.

> 🔴 **PROCÉDURE OBLIGATOIRE AVANT CHAQUE TEST** : lancez d'abord
> **`test_encoders.py`** et **tournez chaque roue à la main** une par une pour
> vérifier que les tics des **4 encodeurs** augmentent. Les 4 encodeurs
> fonctionnent, mais la **carte de commande** a des problèmes intermittents :
> si une roue ne compte pas, **l'odométrie sera fausse** et tous les
> déplacements asservis seront erronés. Repositionnez / vérifiez la carte tant
> que les 4 roues ne comptent pas correctement.

> La plupart des scripts prennent le **port série** en argument
> (`/dev/serial0`, `/dev/serial/by-id/...` sur la Pi, `COMx` sous Windows) et
> ont des options (`--vx`, `--duration`, `--cycles`, …). Lancez‑les avec
> `--help` ou lisez leur docstring en tête de fichier.

## Parcours recommandé (du plus simple au plus complet)

```
0. test_encoders.py      → ⭐ AVANT CHAQUE TEST : vérifier que les 4 roues comptent
1. test_comm.py          → la liaison Pi↔STM32 marche, les moteurs bougent
2. test_encoders.py      → (re)vérifier les encodeurs après toute manip carte
3. test_motor_mapping.py → quelle roue correspond à quel moteur/encodeur
4. test_odom.py          → l'odométrie mesure correctement les vitesses
5. test_move_to.py       → l'asservissement de position converge
6. test_ax12.py          → le servo du prisme se déploie/replie
7. test_xbee.py          → la station Topcon répond via XBee
8. test_codeprincipale_ms → enchaînement déplacement + mesure station
```

## Détail des scripts

### `test_comm.py` — liaison et mouvements de base 🟢
Envoie une séquence de `CMD_SET_VELOCITY` : avant, arrière, (rotation),
chacun pendant `--duration` s, avec un STOP entre chaque. **Le premier test à
lancer** pour vérifier que la chaîne Pi → STM32 → Sabertooth fonctionne.
```bash
python test_comm.py COM6 --vx 0.08 --duration 2 --pause 1
```

### `test_encoders.py` — validation des encodeurs (étape 1) ⭐ À LANCER AVANT CHAQUE TEST
Ne commande **pas** les moteurs : on **pousse / tourne chaque roue à la main**
et on lit la pose et les compteurs. Permet de vérifier : que les **4 encodeurs
comptent** (essentiel — voir l'encadré en haut de page), le sens de chaque
encodeur, l'échelle (`TICKS_PER_METER`), le signe de θ.
**C'est le contrôle de routine indispensable** : la carte de commande étant
intermittente, une roue qui ne compte pas fausse toute l'odométrie.

### `test_motor_mapping.py` — diagnostic du mapping moteurs/encodeurs
Utilise des combinaisons vx/vy qui isolent des **paires de roues** grâce à la
cinématique mecanum (ex. vx=+v, vy=+v → seules FR et RL tournent). Sert à
établir quel Sabertooth pilote quelle roue et à vérifier les polarités. **C'est
ce script qui a fixé le mapping documenté dans `robot_config.h` (2026‑05‑22).**

### `test_odom.py` — validation odométrie en vitesse (étape 2)
Pour chaque direction et chaque vitesse, envoie une consigne, échantillonne
`GET_STATUS`, et calcule le ratio vitesse mesurée / vitesse commandée et la
distance parcourue. Permet de **calibrer `TICKS_PER_METER`**.

### `test_move_to.py` — asservissement de position (étape 3)
Envoie une coordonnée absolue (x, y) et suit la convergence du contrôleur en
pollant `GET_STATUS`. **Commencez par des cibles très petites (≤ 5 cm)** tant
que la calibration n'est pas sûre.

### `test_ax12.py` — servo du prisme 🟢
Déploie puis replie le prisme via `CMD_DEPLOY_PRISM` / `CMD_RETRACT_PRISM`,
avec option de boucle sur N cycles. Vérifie le bon fonctionnement mécanique du
bras.
```bash
python test_ax12.py COM6 --cycles 3 --pause 2.0
```

### `test_robotf.py`, `test_robotf2.py` — tests moteurs interactifs
Tests interactifs des paires de roues en diagonale (isolables via la
cinématique). `test_robotf2` est une évolution de `test_robotf`.

### `test_robotf3.py` — test moteurs automatique
Enchaîne automatiquement 10 mouvements (diagonales, translations, rotations)
sans interaction clavier. Pratique pour une démonstration / vérification rapide.

### `test_robotf4.py` — test automatique **silencieux** ⚠️
Variante de `test_robotf3` qui **n'envoie aucune trame pendant le mouvement**
(pas de polling). Écrit spécifiquement pour **contourner le bug d'overrun
UART** : tout `GET_STATUS` envoyé pendant que le robot bouge risquait de bloquer
la réception du STM32. À lire absolument — sa docstring explique le bug en
détail (voir aussi [passation](../08_Passation/README.md)).

### `test_robotf5.py` — diagnostic bas niveau (firmware patché requis)
Pilote **une seule roue à la fois** via des commandes bas niveau
(`CMD_TEST_WHEEL`, `CMD_RAW_VELOCITY`) qui court‑circuitent PID/cinématique/odom.
⚠️ **Nécessite une version patchée du firmware** ajoutant ces commandes et un
`HAL_UART_ErrorCallback`. Ces commandes ne sont **pas** dans le firmware actuel
(`protocol.h`) — à réintégrer si vous voulez utiliser ce script.

### `test_xbee.py` — diagnostic station Topcon via XBee 🟡
Vérifie la liaison XBee ↔ station : ouverture du port, ACK, trame CSV de mesure.
Calque le protocole de `ms1ax.py`. Voir [Station Topcon](../04_Raspberry_Pi/Station_Topcon.md).

### `test_codeprincipale_ms.py` — enchaînement complet (déplacement + mesure)
Script proche du déroulé d'une mission : pilote la station (`readport`,
commandes) en coordination avec les déplacements. Précurseur du WebSocket de
mission du backend.

### `test_x.py` — banc d'essai station
Petit script de test de communication avec la station (mêmes briques que
`test_codeprincipale_ms.py`). Utilitaire de mise au point.

## Tableau récapitulatif

| Script | Cible | Moteurs ? | Station ? | État |
|--------|-------|-----------|-----------|------|
| `test_comm.py` | liaison + déplacements | oui | non | 🟢 |
| `test_encoders.py` | encodeurs (à la main) | non | non | 🟢 |
| `test_motor_mapping.py` | mapping roues | oui | non | 🟢 |
| `test_odom.py` | odométrie vitesse | oui | non | 🟡 |
| `test_move_to.py` | asservissement position | oui | non | 🟡 |
| `test_ax12.py` | servo prisme | non* | non | 🟢 |
| `test_robotf.py` / `f2` | moteurs interactif | oui | non | 🟡 |
| `test_robotf3.py` | moteurs auto | oui | non | 🟡 |
| `test_robotf4.py` | moteurs auto silencieux | oui | non | 🟢 |
| `test_robotf5.py` | roue par roue | oui | non | 🔴 firmware patché requis |
| `test_xbee.py` | liaison station | non | oui | 🟡 |
| `test_codeprincipale_ms.py` | déplacement + mesure | oui | oui | 🟡 |
| `test_x.py` | station (banc d'essai) | non | oui | 🟡 |

\* `test_ax12.py` commande le servo, pas les roues.

> **Conseil de reprise** : si vous repartez de zéro, refaites le parcours 1→6
> dans l'ordre pour reprendre confiance dans chaque brique avant d'attaquer une
> mission complète.
