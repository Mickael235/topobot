# 6 — Calibration & dépannage

## Partie A — Calibration

Tous les paramètres se règlent dans **`Core/Inc/robot_config.h`**. Après
modification : recompiler et reflasher le STM32.

### A.1 Dimensions du robot

À **mesurer physiquement** et reporter :

| Constante | Signification | Valeur actuelle |
|-----------|---------------|-----------------|
| `WHEEL_RADIUS_M` | rayon d'une roue (m) | `0.076` (à vérifier !) |
| `WHEEL_BASE_LX` | demi‑empattement avant/arrière (m) | `0.110` |
| `WHEEL_BASE_LY` | demi‑empattement gauche/droite (m) | `0.18725` |

> `WHEEL_RADIUS_M` porte la mention « à vérifier ». Mesurez le **diamètre** réel
> d'une roue (galets inclus) et divisez par 2.

### A.2 Encodeurs

| Constante | Rôle |
|-----------|------|
| `ENCODER_PPR` | lignes physiques de l'encodeur (`12`) |
| `ENCODER_COUNTS` | = PPR × 4 (quadrature ×4) |
| `GEAR_RATIO` | rapport de réduction de la boîte (`51.0`) |
| `ENC_POL_*` | polarité de comptage (+1 / −1) par roue |

### A.3 Calibration de la distance (`TICKS_PER_METER`)

C'est **la** calibration la plus importante pour la précision des
déplacements.

Procédure :
1. Reset odométrie (`CMD_RESET_ODOM`).
2. Poussez le robot **bien droit sur 1,00 m** mesuré au mètre (ou commandez un
   déplacement et mesurez le réel).
3. Lisez la valeur `x` affichée par l'odométrie (`test_encoders.py` ou
   `test_odom.py`).
4. Ajustez le multiplicateur `TICKS_PER_METER_CALIB` :
   - si l'odométrie affiche **moins** que la distance réelle → **réduisez** ;
   - si elle affiche **plus** → **augmentez**.

> Actuellement `TICKS_PER_METER_CALIB = 1.0` (remis à 1.0 après la correction du
> mapping encodeurs du 2026‑05‑22). L'ancienne valeur 4.64 compensait un mapping
> erroné. **À recalibrer proprement** une fois les 4 encodeurs réparés.

### A.4 Asservissement de position

| Constante | Rôle | Valeur |
|-----------|------|--------|
| `PID_X_KP`, `PID_Y_KP` | gain proportionnel | `3.0` |
| `VELOCITY_CAP` | plafond de vitesse en sortie | `0.10` m/s |
| `VELOCITY_DEADBAND` | sous ce seuil, sortie forcée à 0 | `0.015` m/s |
| `ACCEL_MAX` | rampe d'accélération | `0.40` m/s² |
| `DIAG_VEL_THRESH` | seuil de correction séquentielle des axes | `0.050` m/s |
| `POS_TOLERANCE_M` | tolérance d'arrivée | `0.015` m |
| `SETTLE_CYCLES` | cycles stables avant de valider l'arrivée | `20` (= 200 ms) |

> `VELOCITY_CAP = 0.10 m/s` a été validé empiriquement : au‑dessus, certaines
> roues ne tournaient pas uniformément. Ne montez pas sans retester.

### A.5 Normalisation moteurs

| Constante | Rôle | Valeur |
|-----------|------|--------|
| `MAX_VX`, `MAX_VY` | vitesse correspondant à 100 % Sabertooth | `0.40` m/s |
| `MIN_MOTOR_NORM` | zone morte moteur (sous ce % → 0) | `0.02` |

> Si certaines roues ne **démarrent pas** à basse consigne, remontez
> `MIN_MOTOR_NORM` à 0.03–0.04.

### A.6 Servo AX12 (positions du prisme)

| Constante | Rôle | Valeur |
|-----------|------|--------|
| `AX12_ID_PRISM` | ID Dynamixel du servo | `2` |
| `AX12_POS_DEPLOY` | position déployée (0…1023) | `700` |
| `AX12_POS_RETRACT` | position rentrée | `200` |

> Ajustez les positions selon la course mécanique réelle du bras du prisme.

---

## Partie B — Dépannage

### Le robot ne bouge pas du tout
- Alimentation puissance : le relais **PA5** doit être actif (firmware le met à
  l'état haut au démarrage). Batterie chargée ?
- LED des Sabertooth : rouge = pas de signal valide. Vérifiez le câblage UART4
  (PC10) et les **adresses DIP** (128 / 129).
- Lancez `test_comm.py` pour isoler la chaîne logicielle.

### Une seule roue tourne / le robot part de travers
- Consigne sous le seuil de démarrage moteur → augmentez `MIN_MOTOR_NORM` ou
  travaillez au‑dessus de `VELOCITY_DEADBAND`.
- Vérifiez les polarités `POL_*` (`mecanum.c`) avec `test_motor_mapping.py`.

### Une roue tourne à l'envers
- Inversez le signe `POL_xx` correspondant dans `mecanum.c` (plutôt que les
  fils).

### L'odométrie est fausse / dérive
- **Cause n°1 (la plus fréquente)** : la **carte de commande** a un faux contact
  et une ou plusieurs roues **ne comptent plus**. Les 4 encodeurs sont bons,
  mais si la carte ne transmet pas les tics, l'odométrie est fausse.
  → **Avant chaque test**, lancez `test_encoders.py`, tournez les 4 roues à la
  main et confirmez que chacune incrémente. Repositionnez / vérifiez la carte
  tant que ce n'est pas le cas.
- Rappel firmware : `ODOM_REAR_ONLY=1` n'exploite que RL/RR et **force θ à 0**.
  Une fois les 4 roues confirmées fiables, vous pouvez repasser ce define à `0`.
- Vérifiez ensuite `TICKS_PER_METER` (voir A.3) et les polarités encodeurs.

### La réception STM32 se bloque (plus aucune réponse) ⚠️
- **Bug d'overrun UART connu** : pendant les ≈17 ms d'envoi aux Sabertooth, un
  octet reçu de la Pi peut être perdu et bloquer la réception (flag ORE non
  nettoyé).
- **Contournement immédiat** : appuyer sur le **reset** de la Nucleo.
- **Contournements logiciels** : ne pas envoyer de `GET_STATUS` pendant un
  mouvement (cf. `test_robotf4.py`), ou ajouter un `HAL_UART_ErrorCallback` qui
  nettoie l'erreur et relance la réception (cf. besoin de `test_robotf5.py`).
- Correctif de fond proposé dans la [passation](08_Passation/README.md).

### L'ARU se déclenche tout seul
- Parasites moteurs sur PB14. Le firmware filtre déjà (debounce 50 ms) ; si ça
  persiste, vérifiez le câblage / blindage, ou augmentez `ARU_DEBOUNCE_CYCLES`
  dans `robot_control.c`.

### La station Topcon ne mesure pas
- Vérifiez le port (`/dev/ttyUSB0`), le XBee, le débit (9600).
- Lancez `test_xbee.py` puis `python ms1ax.py /dev/ttyUSB0 mes`.
- La station doit **viser le prisme** ; en tracking elle peut le perdre si le
  robot va trop vite.
- Ne raccourcissez pas les `time.sleep` des séquences Topcon.

### Le frontend n'affiche rien / page blanche
- `frontend/dist` existe ? Sinon `npm run build`.
- Le backend tourne et est joignable (`http://<IP>:8000`) ?
- Ouvrez la console du navigateur (F12) pour voir les erreurs WebSocket/REST.
