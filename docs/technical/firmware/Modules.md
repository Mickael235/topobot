# Modules du firmware — fichier par fichier

Tous les fichiers ci‑dessous sont dans `Core/Src/` (sources) et `Core/Inc/`
(en‑têtes). Pour chaque module : son rôle, ses points d'entrée et les pièges.

---

## `main.c`

Point d'entrée. Responsabilités :
- Initialise les périphériques (générés par CubeMX) : ADC, TIM1‑4/8, UART4/5,
  USART2, GPIO.
- Initialise **TIM6 à la main** (`MX_TIM6_Init`) — base de temps 100 Hz, **non
  générée par CubeMX**.
- Lance les sous‑systèmes : `RobotControl_Init()`, `Protocol_Init()`,
  `AX12_Init()`, active l'alimentation puissance (PA5).
- Boucle `while(1)` : `Protocol_Process()` + `RobotControl_Update()` cadencé par
  le flag `velocity_tick`.
- Contient les callbacks HAL : `HAL_TIM_PeriodElapsedCallback` (TIM6 → flag) et
  `HAL_UART_RxCpltCallback` (USART2 → `Protocol_UART_RxCallback`).

Variables globales partagées : `g_vx`, `g_vy`, `g_wz` (consigne de vitesse
courante), `velocity_tick`.

> ⚠️ Si vous régénérez depuis le `.ioc`, vérifiez que `MX_TIM6_Init`, l'init des
> sous‑systèmes et les callbacks sont **toujours là** (ils sont dans des blocs
> `USER CODE`, donc normalement préservés).

---

## `robot_config.h` ⭐

**Le fichier de réglage central.** Aucune logique, que des `#define`. Regroupe
mapping/polarités encodeurs, UART, AX12, dimensions du robot, calibration
odométrie, gains et limites de l'asservissement, fréquence de boucle.
**Commencez toujours par ici pour régler un comportement.** Voir aussi
[Calibration](../06_Calibration_Depannage.md).

---

## `protocol.c` / `protocol.h`

Dialogue série avec la Raspberry Pi (USART2). Réception en interruption octet
par octet, reconstruction de trame par machine d'état, traitement dans la
boucle principale. Décode les commandes et appelle les modules concernés
(`RobotControl_*`, `Mecanum_*`, `Odom_*`, `AX12_*`), renvoie ACK/DONE/STATUS/ERROR.
Détail complet :
[Protocole de communication](Protocole_Communication.md).

---

## `robot_control.c` / `robot_control.h`

Superviseur de haut niveau : **machine d'états** + **asservissement de
position** (proportionnel pur saturé sur x et y, voir
[README firmware §3.5](README.md#35-stratégie-dasservissement-important)).

Points d'entrée :
- `RobotControl_Init()` — init PID/odom/mecanum, état IDLE.
- `RobotControl_Update()` — appelé à 100 Hz, fait tourner odométrie + ARU +
  asservissement.
- `RobotControl_MoveTo(x, y, θ)` — nouvelle cible (θ ignoré).
- `RobotControl_EmergencyStop()` / `RobotControl_Reset()`.
- `RobotControl_SetGains(...)` / `RobotControl_SetGridLimits(...)`.

Gère aussi : **debounce de l'ARU**, **keepalive Sabertooth** (ré‑envoie un stop
toutes les 500 ms à l'arrêt pour ne pas perdre la liaison), **vitesses
manuelles** en IDLE (via `g_vx`/`g_vy`).

> La vérification des **limites de grille** (`in_grid`) est actuellement
> **désactivée** (`return true;`). À réactiver quand l'odométrie sera fiable.

---

## `odometry.c` / `odometry.h`

Calcule la pose (x, y, θ) à partir des encodeurs. Lecture des compteurs avec
gestion de l'overflow 16 bits, application des polarités, cinématique directe
mecanum, intégration en repère monde (RK2 / point milieu).

⚠️ **Mode `ODOM_REAR_ONLY`** (activé dans `robot_config.h`) : l'odométrie
n'utilise que les 2 encodeurs arrière (RL, RR). Conséquence : **θ est forcé à
0** (avec 2 encodeurs seulement, le calcul d'angle génère une fausse rotation
pendant les déplacements latéraux). C'est acceptable car la rotation n'est ni
commandée ni asservie.

> **Important — réalité matérielle vs commentaires du code** : les commentaires
> de ce fichier et de `robot_config.h` indiquent que FL/FR seraient « hors
> service ». En pratique, **les 4 encodeurs fonctionnent** ; c'est la **carte de
> commande** qui peut, par intermittence, empêcher certains tics d'être comptés.
> D'où la **procédure obligatoire** : vérifier les 4 roues avec
> `test_encoders.py` avant chaque test. Une fois le comptage des 4 roues
> confirmé fiable, `ODOM_REAR_ONLY` peut être repassé à **0** pour exploiter les
> 4 encodeurs et restaurer l'odométrie de θ. Voir
> [passation](../08_Passation/README.md).

Points d'entrée : `Odom_Init`, `Odom_Update(dt)`, `Odom_Reset`,
`Odom_SetPose`, `Odom_GetState`.

---

## `mecanum.c` / `mecanum.h`

Bas niveau moteurs. Convertit une consigne (vx, vy) en vitesses de roues
(cinématique inverse mecanum **sans rotation**), normalise, applique les
polarités de câblage (`POL_*`), et envoie les **paquets Sabertooth** sur UART4.

Cinématique inverse utilisée (translation pure) :
```
w_FL = (vx − vy) / r        w_FR = (vx + vy) / r
w_RL = (vx + vy) / r        w_RR = (vx − vy) / r
```

Points d'entrée : `Mecanum_Init`, `Mecanum_SetVelocity(vx, vy)`,
`Mecanum_Stop`.

> L'envoi est **bloquant** (`HAL_UART_Transmit`) : 4 paquets × 4 octets à
> 9600 bauds ≈ 17 ms. C'est la source du bug d'overrun (voir
> [passation](../08_Passation/README.md)).

---

## `pid.c` / `pid.h`

Régulateur PID générique (anti‑windup + filtrage dérivé). **Dans la version
actuelle, seuls les gains KP sont réellement utilisés** par `robot_control.c`
(les termes I et D sont neutralisés → proportionnel pur). Le module reste
complet et prêt si vous voulez réactiver un vrai PID.

Points d'entrée : `PID_Init`, `PID_Compute(error, dt)`, `PID_Reset`,
`PID_SetGains`.

---

## `ax12.c` / `ax12.h`

Pilote du servo **AX12 Dynamixel** (UART5, 1 Mbps, protocole Dynamixel v1).
Active le couple à l'init, écrit la position et la vitesse cibles.

Points d'entrée :
- `AX12_Init()` — reconfigure UART5 à 1 Mbps, active le couple.
- `AX12_Deploy()` / `AX12_Retract()` — positions par défaut (700 / 200).
- `AX12_DeployCustom(position, vitesse)` — déploiement paramétrable.
- `AX12_SetGoalPosition(id, pos)` / `AX12_SetMovingSpeed(id, speed)`.

---

## Fichiers générés (ne pas modifier sauf nécessité)

| Fichier | Rôle |
|---------|------|
| `stm32f4xx_it.c` | Handlers d'interruptions (IRQ). Ajoutez les vôtres dans les blocs USER CODE. |
| `stm32f4xx_hal_msp.c` | Configuration bas niveau des périphériques (horloges, GPIO, NVIC). |
| `system_stm32f4xx.c`, `syscalls.c`, `sysmem.c` | Démarrage système / runtime C. |
| `startup_stm32f446retx.s` | Vecteur d'interruptions / démarrage (assembleur). |
| `Drivers/` | Bibliothèque HAL STMicroelectronics. |
