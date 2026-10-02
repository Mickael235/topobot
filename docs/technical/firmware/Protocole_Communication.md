# Protocole de communication Pi ↔ STM32

Liaison **USART2 à 115200 bauds**. Protocole binaire maison, défini des deux
côtés :
- côté STM32 : `Core/Inc/protocol.h` et `Core/Src/protocol.c`
- côté Pi : `topobot/topobot/backend/main.py` (section « PROTOCOLE »)

> Les deux fichiers doivent rester **synchronisés** : si vous ajoutez une
> commande, modifiez les deux.

## Format d'une trame

```
┌────────┬──────┬────────┬───────────────┬──────┐
│ START  │ CMD  │  LEN   │  PAYLOAD...    │ CRC  │
│ 0xAA   │ 1 o. │ 1 o.   │  LEN octets    │ 1 o. │
└────────┴──────┴────────┴───────────────┴──────┘
```

- **START** = `0xAA` (octet de synchronisation).
- **CMD** = code de commande ou de réponse (voir tables ci‑dessous).
- **LEN** = longueur du payload (0 à 63).
- **PAYLOAD** = données (souvent des `float` 32 bits little‑endian).
- **CRC** = XOR de `CMD`, `LEN` et de tous les octets du payload.

```python
def crc(cmd, payload):
    c = cmd ^ len(payload)
    for b in payload:
        c ^= b
    return c & 0xFF
```

## Commandes (Raspberry Pi → STM32)

| Code | Nom | Payload | Effet |
|------|-----|---------|-------|
| `0x01` | `CMD_MOVE_TO` | x, y, θ (3 floats) | Déplacement asservi vers (x, y). θ ignoré. |
| `0x02` | `CMD_STOP` | — | Arrêt moteurs + retour IDLE. |
| `0x03` | `CMD_RESET_ODOM` | — | Remet pose (x,y,θ) et encodeurs à zéro. |
| `0x04` | `CMD_GET_STATUS` | — | Demande la télémétrie (voir RESP_STATUS). |
| `0x05` | `CMD_SET_GRID` | (max_x, max_y) | Définit les limites de la grille de sécurité. |
| `0x06` | `CMD_SET_GAINS` | gains | Modifie les gains de l'asservissement à chaud. |
| `0x07` | `CMD_SET_VELOCITY` | vx, vy, wz (3 floats) | Vitesse manuelle directe (wz ignoré). |
| `0x10` | `CMD_DEPLOY_PRISM` | *(optionnel : pos u16 + vitesse u16)* | Déploie le prisme (servo AX12). |
| `0x11` | `CMD_RETRACT_PRISM` | — | Replie le prisme. |

> `CMD_DEPLOY_PRISM` accepte un payload optionnel de 4 octets
> `[position(u16 LE)][vitesse(u16 LE)]` pour un déploiement paramétrable ; sans
> payload, il utilise la position/vitesse par défaut de `robot_config.h`.

## Réponses (STM32 → Raspberry Pi)

| Code | Nom | Payload | Sens |
|------|-----|---------|------|
| `0xA1` | `RESP_ACK` | (cmd echo) | Commande reçue et acceptée. |
| `0xA2` | `RESP_DONE` | x, y, θ | Mouvement terminé (cible atteinte). |
| `0xA3` | `RESP_STATUS` | structure télémétrie | Réponse à `GET_STATUS`. |
| `0xA4` | `RESP_ERROR` | code erreur (1 o.) | Erreur. |

### Codes d'erreur

| Code | Nom | Cause |
|------|-----|-------|
| `0x01` | `ERR_CRC` | CRC invalide ou payload trop court. |
| `0x02` | `ERR_UNKNOWN_CMD` | Commande inconnue. |
| `0x03` | `ERR_BUSY` | Robot occupé. |
| `0x04` | `ERR_OUT_BOUNDS` | Sortie de la grille de sécurité. |
| `0x05` | `ERR_ARU` | Arrêt d'urgence actif. |
| `0x06` | `ERR_AX12` | Échec communication avec le servo. |

## Structure de télémétrie (`RESP_STATUS`)

Format Python : `"<fffiiiiffffffB"` (little‑endian). Champs, dans l'ordre :

| Champ | Type | Description |
|-------|------|-------------|
| vx_cmd, vy_cmd, wz_cmd | float ×3 | Consigne de vitesse courante |
| e1, e2, e3, e4 | int32 ×4 | Compteurs encodeurs bruts (FL, FR, RL, RR) |
| x, y, theta | float ×3 | Pose odométrie (repère monde) |
| vx_meas, vy_meas, wz_meas | float ×3 | Vitesses mesurées (repère robot) |
| state | uint8 | État machine (0=IDLE … 5=ERROR) |

> ⚠️ L'ordre des encodeurs dans la structure correspond aux handles
> `ENC_FL/FR/RL/RR` (TIM4, TIM2, TIM8, TIM3). Comme FL et FR sont hors service,
> e1 et e2 restent figés ; e3 (RL) et e4 (RR) bougent.

## Réception côté STM32 : machine d'état octet par octet

`protocol.c` reçoit **un octet à la fois en interruption**
(`HAL_UART_Receive_IT`) et reconstruit la trame avec une petite machine d'état
(`RX_WAIT_START → RX_WAIT_CMD → RX_WAIT_LEN → RX_DATA → RX_WAIT_CRC`). Quand le
CRC est bon, un flag `frame_ready` est levé et la trame est traitée dans la
boucle principale par `Protocol_Process()` (jamais en interruption).

> **Bug connu lié à ce mécanisme** : si une trame arrive pendant que le STM32
> est occupé à envoyer aux Sabertooth (17 ms), un octet peut être perdu
> (overrun). Détails et contournements dans la
> [passation](../08_Passation/README.md). Plusieurs scripts de test
> (`test_robotf4.py`, `test_robotf5.py`) ont été écrits spécifiquement pour
> éviter ou diagnostiquer ce problème.

## Exemple : envoyer un MOVE_TO depuis Python

```python
import struct, serial
def crc(cmd, p): 
    c = cmd ^ len(p)
    for b in p: c ^= b
    return c & 0xFF
def frame(cmd, p=b""): 
    return bytes([0xAA, cmd, len(p)]) + p + bytes([crc(cmd, p)])

ser = serial.Serial("COM6", 115200, timeout=1)
payload = struct.pack("<fff", 0.10, 0.0, 0.0)   # aller à x=10 cm
ser.write(frame(0x01, payload))                  # CMD_MOVE_TO
```
