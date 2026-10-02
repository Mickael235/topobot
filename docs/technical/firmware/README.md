# 3 — Firmware STM32

Le firmware est le programme temps réel embarqué dans le STM32 F446RE. Il vit
dans le dossier `Core/` (le dossier `Drivers/` est la bibliothèque HAL générée,
à ne pas modifier).

Voir aussi :
- [Protocole de communication](Protocole_Communication.md) — trames Pi ↔ STM32
- [Modules firmware](Modules.md) — description fichier par fichier

## 3.1 Environnement et compilation

- **IDE** : STM32CubeIDE (basé sur Eclipse + GCC ARM).
- **Ouvrir le projet** : *File ▸ Open Projects from File System…* → dossier
  `ProjetRobot`. Le `.cproject` / `.project` sont déjà présents.
- **Configuration matérielle** : `ProjetRobot.ioc` (CubeMX). Double‑cliquez
  pour modifier le brochage / les périphériques ; CubeMX régénère ensuite les
  fonctions `MX_*_Init` dans `main.c`.
- **Compiler** : marteau 🔨 (*Build*). Le binaire `.elf` est produit dans
  `Debug/`.
- **Flasher / déboguer** : insecte vert 🐞 (*Debug*) ou *Run*. La carte Nucleo
  embarque un ST‑Link, le flash se fait par USB sans matériel supplémentaire.

> **Règle d'or CubeMX** : tout code que vous écrivez doit rester entre les
> balises `/* USER CODE BEGIN ... */` et `/* USER CODE END ... */`. Tout ce qui
> est en dehors est **écrasé** à chaque régénération depuis le `.ioc`.

## 3.2 Architecture logicielle

Le firmware est organisé en couches, du bas niveau au haut niveau :

```
        ┌─────────────────────────────────────────────┐
        │              main.c (boucle while)           │
        │  - init des périphériques                    │
        │  - TIM6 100 Hz → flag velocity_tick          │
        │  - appelle Protocol_Process + RobotControl   │
        └───────────────┬───────────────┬──────────────┘
                        │               │
        ┌───────────────▼──┐   ┌────────▼───────────────┐
        │  protocol.c       │   │  robot_control.c       │
        │  (dialogue Pi)    │   │  (machine d'états +    │
        │                   │   │   asservissement x,y)  │
        └───────────────────┘   └──┬─────────┬───────┬───┘
                                   │         │       │
                       ┌───────────▼─┐ ┌─────▼────┐ ┌▼────────┐
                       │ odometry.c   │ │ pid.c    │ │ mecanum.c│
                       │ (pose x,y,θ) │ │ (régul.) │ │ (roues + │
                       │              │ │          │ │ Sabertooth)│
                       └──────────────┘ └──────────┘ └──────────┘

                       ax12.c  ←  pilotage servo prisme (appelé par protocol.c)
```

## 3.3 La boucle de contrôle 100 Hz

C'est le cœur temps réel. Mécanisme :

1. **TIM6** est configuré à **100 Hz** (`main.c`, `MX_TIM6_Init`). À chaque
   débordement, son interruption met le flag `velocity_tick = 1` (et **rien
   d'autre** : aucun travail bloquant en interruption).
2. La boucle `while(1)` de `main.c` :
   - traite les trames reçues de la Pi (`Protocol_Process()`),
   - et quand `velocity_tick` est levé, appelle `RobotControl_Update()`.
3. `RobotControl_Update()` (à 100 Hz) :
   - met à jour l'**odométrie** (lecture des encodeurs),
   - filtre l'**ARU**,
   - selon l'état, calcule les vitesses (asservissement) et les envoie aux
     **Sabertooth** via `Mecanum_SetVelocity()`.

> **Pourquoi l'envoi Sabertooth se fait en boucle principale et pas en
> interruption ?** L'envoi de 4 paquets à 9600 bauds prend ≈ 17 ms. Si on le
> faisait dans l'interruption TIM6 (priorité haute), il masquerait la réception
> USART2 pendant 17 ms → risque d'**overrun** (octets perdus, réception bloquée).
> En le faisant dans la boucle principale, la réception peut préempter
> normalement. **Ce point reste fragile** — voir
> [passation](../08_Passation/README.md).

## 3.4 La machine d'états du robot

Définie dans `robot_control.h` (`RobotState_t`) :

| État | Signification |
|------|---------------|
| `ROBOT_IDLE` | En attente. Peut recevoir des vitesses manuelles (`CMD_SET_VELOCITY`). |
| `ROBOT_MOVING` | Déplacement asservi en cours vers une cible (x, y). |
| `ROBOT_SETTLING` | (réservé) stabilisation sur la cible. |
| `ROBOT_DONE` | Cible atteinte et stabilisée. |
| `ROBOT_ESTOP` | Arrêt d'urgence (bouton ARU). |
| `ROBOT_ERROR` | Erreur (hors limites de grille, etc.). |

Transitions principales :
- `CMD_MOVE_TO` → `MOVING` ; convergence (pose stable `SETTLE_CYCLES` cycles) → `DONE`.
- ARU appuyé → `ESTOP` (depuis n'importe quel état).
- `CMD_STOP` / `CMD_RESET_ODOM` → retour à `IDLE`.

## 3.5 Stratégie d'asservissement (important)

L'asservissement de position est un **proportionnel pur saturé avec zone
morte**, pas un vrai PID. Choix volontaire, documenté dans `robot_control.c` :

```
vx = saturation(KP · erreur_x , ±VELOCITY_CAP) ; si |vx| < zone_morte → 0
vy = saturation(KP · erreur_y , ±VELOCITY_CAP) ; si |vy| < zone_morte → 0
```

Raisons :
1. **Pas d'intégrale (I)** → pas de windup, pas de dérive à l'arrêt.
2. **Pas de dérivée (D)** → pas de pic au démarrage qui saturait la commande.
3. Garder la sortie dans la plage `[zone_morte, VELOCITY_CAP]` où **toutes les
   roues répondent uniformément** (régime validé empiriquement vers 0,10 m/s).

S'y ajoutent :
- une **rampe d'accélération** (slew rate) pour démarrer en douceur,
- une **correction séquentielle des axes** : quand vx et vy sont proches en
  magnitude, on ne garde que l'axe dominant (sinon la cinématique mecanum met
  une paire de roues à ≈0 → déséquilibre).

Tous les paramètres (gains, plafonds, seuils, tolérances) sont centralisés dans
**`robot_config.h`** — c'est le fichier à éditer pour régler le comportement.

## 3.6 Fichier de configuration : `robot_config.h`

**À lire en entier.** Il regroupe :
- mapping et polarités des encodeurs,
- handles UART et adresses Sabertooth,
- paramètres AX12 (ID, positions, débit),
- dimensions du robot (rayon de roue, empattements),
- calibration de l'odométrie (`TICKS_PER_METER`),
- gains et limites de l'asservissement,
- fréquence de la boucle de contrôle.

> ⚠️ Plusieurs constantes portent la mention « à recalibrer » ou « à mesurer ».
> Voir [Calibration](../06_Calibration_Depannage.md).
