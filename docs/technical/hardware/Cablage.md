# Câblage détaillé

Détail des bus et liaisons, avec les pièges rencontrés cette année.

## 1. Bus Sabertooth (moteurs) — UART4 / PC10

Les **deux Sabertooth partagent le même fil** de données série, branché sur
**PC10 (UART4_TX)** du STM32. C'est une liaison **unidirectionnelle** (le STM32
parle, les Sabertooth écoutent) en mode « **Packetized Serial** » à **9600
bauds**.

```
  STM32 PC10 (UART4_TX) ──┬──► S1 Sabertooth #1  (DIP réglés sur adresse 128) ─► moteurs FL + RL
                          └──► S1 Sabertooth #2  (DIP réglés sur adresse 129) ─► moteurs FR + RR
```

- La distinction entre les deux contrôleurs se fait par **l'adresse** envoyée
  en début de trame (128 ou 129), réglée sur les **micro‑switchs (DIP)** de
  chaque Sabertooth.
- Format d'une trame : `[Adresse] [Commande] [Valeur] [Checksum]`,
  `Checksum = (Adresse + Commande + Valeur) & 0x7F`. Voir `mecanum.c`.
- Sur chaque Sabertooth : moteur **A** = commandes 0/1 (avant/arrière),
  moteur **B** = commandes 4/5.

### Polarité de câblage des moteurs

Selon le sens de branchement physique des moteurs, certains tournent à
l'envers de la commande. C'est corrigé **logiciellement** dans `mecanum.c` :

```c
#define POL_FL  (-1)   // moteur câblé inversé
#define POL_FR  (+1)
#define POL_RL  (+1)
#define POL_RR  (-1)   // moteur câblé inversé
```

> Si vous rebranchez/remplacez un moteur et qu'il part dans le mauvais sens,
> changez le signe correspondant ici **plutôt que d'inverser les fils**.

## 2. Encodeurs — TIM2 / TIM3 / TIM4 / TIM8

Chaque encodeur a 2 voies (A et B) lues en **quadrature ×4** par un timer.
Voir le tableau dans [Brochage_STM32.md](Brochage_STM32.md).

Pièges :
- Côté **gauche**, les voies A/B sont **permutées** sur les canaux du timer →
  le compteur décompte quand la roue avance. Corrigé par `ENC_POL_*` négatif.
- **Les 4 encodeurs fonctionnent**, mais le comptage dépend de l'état de la
  **carte de commande** (interface des encodeurs) : si elle a un faux contact,
  une ou plusieurs roues cessent de compter et l'odométrie devient fausse.
  → **Vérifier les 4 roues avec `test_encoders.py` avant chaque test** (tourner
  chaque roue à la main, confirmer que les tics augmentent).

## 3. Servo AX12 (prisme) — UART5 / PC12

- Le servo **AX12 Dynamixel** est sur **UART5** (PC12 = TX, PD2 = RX), à
  **1 Mbps** (débit usine du servo). UART dédié → pas de conflit avec la Pi.
- Protocole **Dynamixel v1** : `FF FF [ID] [LEN] [INSTR] [PARAMS] [CHK]`.
  Voir `ax12.c`.
- L'**ID** du servo est `2` (`AX12_ID_PRISM` dans `robot_config.h`).
- Positions : `700` = déployé, `200` = rentré (échelle 0…1023 ↔ 0…300°).
  Ajustez ces valeurs si la course mécanique du prisme change.

> Le STM32 initialise UART5 à 115200 par CubeMX, puis `AX12_Init()` la
> **reconfigure à 1 Mbps**. Si vous changez le débit du servo, changez
> `AX12_BAUDRATE`.

## 4. Liaison Raspberry Pi ↔ STM32 — USART2 / PA2‑PA3

- **USART2** à **115200 bauds**, full‑duplex.
- Côté Pi, la connexion passe par l'**USB** (port série virtuel du ST‑Link de
  la Nucleo). Le chemin stable côté Pi ressemble à :
  `/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_...-if02`
  (défini dans `backend/main.py`, variable `STM32_PORT`).
- Protocole binaire maison : voir
  [Protocole de communication](../03_Firmware_STM32/Protocole_Communication.md).

## 5. Arrêt d'urgence (ARU) — PB14

- Entrée GPIO **PB14** avec **pull‑up** : actif à l'**état bas** (bouton
  appuyé → 0 V).
- Le firmware applique un **debounce** (5 lectures basses consécutives à
  100 Hz = 50 ms) pour ignorer les parasites électromagnétiques des moteurs.
  Voir `robot_control.c` (`aru_filter_step`).
- Quand l'ARU est actif : arrêt immédiat des moteurs, passage en état `ESTOP`,
  envoi de `ERR_ARU` à la Pi.

## 6. Liaison Raspberry Pi ↔ Station Topcon — XBee

- La station Topcon est reliée par **radio XBee**. Côté Pi, le XBee est branché
  en **USB via un adaptateur FTDI** → apparaît comme `/dev/ttyUSB0` (variable
  `TOPCON_PORT` dans `backend/main.py`).
- Débit : **9600 bauds**.
- Voir [Station Topcon](../04_Raspberry_Pi/Station_Topcon.md) pour les
  commandes.

> Vérifier le nom exact du port côté Pi avec `ls /dev/serial/by-id/` ou
> `dmesg | grep ttyUSB` — il peut changer selon l'ordre de branchement USB.

## 7. Relais d'alimentation puissance — PA5

- **PA5** (`Power_enable_stm`) commande un relais qui autorise le courant vers
  les Sabertooth.
- Mis à l'état **haut** par le firmware au démarrage (`main.c`). Tant que PA5
  est bas, les moteurs ne sont pas alimentés.
