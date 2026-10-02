#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
test_moteurs_auto.py — version SILENCIEUSE
==========================================
Test AUTOMATIQUE des moteurs mecanum, sans interaction clavier.

Différence importante avec la version précédente :
  → Pendant le mouvement, AUCUNE trame n'est envoyée au STM32.
  → Pas de polling GET_STATUS pendant la durée du test.

Pourquoi : le firmware STM32 actuel a une lacune connue.
  • TIM6 (priorité NVIC 1) appelle Mecanum_SetVelocity qui fait 4 transmits
    UART4 bloquants à 9600 bauds → ≈ 17 ms par cycle TIM6.
  • USART2_RX (priorité NVIC 2) est MASQUÉ pendant ces 17 ms.
  • Si une trame arrive du Pi pendant ce blocage, le registre RX overrun,
    le flag ORE n'est pas clearé par la HAL → réception bloquée jusqu'au
    prochain reset du STM32.
  • Donc tout GET_STATUS envoyé pendant que le robot est MOVING provoque
    un overrun → le STOP final n'est jamais reçu → robot bloqué sur la
    consigne du premier test.

Le contournement (sans toucher au firmware) :
  • Pas de communication pendant les `--duration` secondes du mouvement
  • STOP envoyé plusieurs fois en fin de test pour franchir la fenêtre TIM6
  • Vérification de la responsivité du STM32 entre les tests

Usage :
  python3 test_moteurs_auto.py [port] [--dist 0.10] [--duration 10]
  python3 test_moteurs_auto.py /dev/serial0
  python3 test_moteurs_auto.py /dev/ttyACM0 --dist 0.08 --duration 8

Interruption : Ctrl+C → STOP × 5 et sortie propre.

Prérequis : pip install pyserial
"""

import argparse
import math
import serial
import signal
import struct
import sys
import time
from datetime import datetime

# ─────────────────────────────────────────────────────────────
# PROTOCOLE STM32
# ─────────────────────────────────────────────────────────────

PROTO_START    = 0xAA
CMD_MOVE_TO    = 0x01
CMD_STOP       = 0x02
CMD_RESET_ODOM = 0x03
CMD_GET_STATUS = 0x04

RESP_ACK    = 0xA1
RESP_DONE   = 0xA2
RESP_STATUS = 0xA3
RESP_ERROR  = 0xA4

STATE_NAMES = {0:"IDLE", 1:"MOVING", 2:"SETTLING", 3:"DONE", 4:"ESTOP", 5:"ERROR"}
ERR_NAMES   = {
    0x01:"ERR_CRC", 0x02:"ERR_UNKNOWN_CMD", 0x03:"ERR_BUSY",
    0x04:"ERR_OUT_BOUNDS", 0x05:"ERR_ARU", 0x06:"ERR_AX12",
}

# ─────────────────────────────────────────────────────────────
# COULEURS ANSI
# ─────────────────────────────────────────────────────────────

RST  = "\033[0m"
BOLD = "\033[1m"
DIM  = "\033[2m"
GRN  = "\033[92m"
RED  = "\033[91m"
YLW  = "\033[93m"
CYN  = "\033[96m"
WHT  = "\033[97m"
BLU  = "\033[94m"
MAG  = "\033[95m"

# ─────────────────────────────────────────────────────────────
# LOGS
# ─────────────────────────────────────────────────────────────

def ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def log(symbol, color, msg):
    print(f"{DIM}[{ts()}]{RST} {color}{symbol}  {msg}{RST}", flush=True)

def ok(msg):      log("v", GRN, msg)
def err(msg):     log("x", RED, msg)
def warn(msg):    log("!", YLW, msg)
def info(msg):    log(".", CYN, msg)
def cmd_log(msg): log(">", BLU, msg)
def sep():
    print(f"{DIM}{'-' * 62}{RST}", flush=True)

# ─────────────────────────────────────────────────────────────
# DÉFINITION DES TESTS
# ─────────────────────────────────────────────────────────────

def make_tests(d):
    """d = distance par axe (m)."""
    ang = math.radians(25.0)

    return [
        dict(id=1, cat="diag", pair="FL+RR", label="DIAGONALE FL + RR --- AVANT",
             expect="FL+RR tournent vers l'AVANT  -  FR et RL strictement IMMOBILES",
             x= d,   y=-d,  th=0.0),
        dict(id=2, cat="diag", pair="FR+RL", label="DIAGONALE FR + RL --- AVANT",
             expect="FR+RL tournent vers l'AVANT  -  FL et RR strictement IMMOBILES",
             x= d,   y= d,  th=0.0),
        dict(id=3, cat="diag", pair="FL+RR", label="DIAGONALE FL + RR --- ARRIERE",
             expect="FL+RR tournent vers l'ARRIERE  -  FR et RL immobiles",
             x=-d,   y= d,  th=0.0),
        dict(id=4, cat="diag", pair="FR+RL", label="DIAGONALE FR + RL --- ARRIERE",
             expect="FR+RL tournent vers l'ARRIERE  -  FL et RR immobiles",
             x=-d,   y=-d,  th=0.0),
        dict(id=5, cat="move", pair="ALL", label="AVANT",
             expect="Les 4 roues tournent vers l'AVANT a vitesse egale",
             x= d,   y=0.0, th=0.0),
        dict(id=6, cat="move", pair="ALL", label="ARRIERE",
             expect="Les 4 roues tournent vers l'ARRIERE",
             x=-d,   y=0.0, th=0.0),
        dict(id=7, cat="move", pair="LAT", label="LATERAL +y",
             expect="2 roues en avant, 2 en arriere (opposition diagonale)",
             x=0.0,  y= d,  th=0.0),
        dict(id=8, cat="move", pair="LAT", label="LATERAL -y",
             expect="Opposition diagonale inversee",
             x=0.0,  y=-d,  th=0.0),
        dict(id=9, cat="rot",  pair="ROT", label="ROTATION theta>0",
             expect="Le robot pivote sur lui-meme (sens 1)",
             x=0.0,  y=0.0, th= ang),
        dict(id=10, cat="rot", pair="ROT", label="ROTATION theta<0",
             expect="Le robot pivote sur lui-meme (sens inverse)",
             x=0.0,  y=0.0, th=-ang),
    ]

TOTAL = 10

# ─────────────────────────────────────────────────────────────
# PROTOCOLE — ENCODE / DECODE
# ─────────────────────────────────────────────────────────────

def crc(cmd, payload):
    c = cmd ^ len(payload)
    for b in payload:
        c ^= b
    return c & 0xFF

def build(cmd, payload=b''):
    return bytes([PROTO_START, cmd, len(payload)]) + payload + bytes([crc(cmd, payload)])

def send(ser, cmd, payload=b''):
    frame = build(cmd, payload)
    ser.write(frame)
    ser.flush()
    return frame

def recv(ser, timeout=1.0, expect_resp=None):
    ser.timeout = 0.05
    deadline = time.time() + timeout
    while time.time() < deadline:
        b = ser.read(1)
        if not b or b[0] != PROTO_START:
            continue
        hdr = ser.read(2)
        if len(hdr) != 2:
            continue
        resp, ln = hdr[0], hdr[1]
        if ln > 64:
            continue
        rest = ser.read(ln + 1)
        if len(rest) != ln + 1:
            continue
        data, crc_rx = rest[:ln], rest[ln]
        if crc_rx != crc(resp, data):
            continue
        if expect_resp is not None and resp != expect_resp:
            continue
        return resp, data
    return None

def do_reset(ser):
    frame = send(ser, CMD_RESET_ODOM)
    cmd_log(f"RESET_ODOM  [{frame.hex(' ')}]")
    r = recv(ser, 0.6, expect_resp=RESP_ACK)
    if r:
        ok("RESET_ODOM ACK")
        return True
    warn("RESET_ODOM sans reponse")
    return False

def do_move(ser, x, y, th_rad):
    payload = struct.pack('<fff', x, y, th_rad)
    frame = send(ser, CMD_MOVE_TO, payload)
    cmd_log(f"MOVE_TO  x={x:+.3f}m  y={y:+.3f}m  th={math.degrees(th_rad):+.1f}deg  [{frame.hex(' ')}]")
    r = recv(ser, 1.0)
    if r is None:
        err("Aucune reponse au MOVE_TO")
        return False
    code, data = r
    if code == RESP_ACK:
        ok("MOVE_TO ACK --- robot en route")
        return True
    if code == RESP_ERROR:
        e = ERR_NAMES.get(data[0] if data else 0, "?")
        err(f"MOVE_TO REFUSE : {e}")
    else:
        warn(f"Reponse inattendue : 0x{code:02X}")
    return False

def do_status_quick(ser):
    """GET_STATUS avec timeout court, retourne None si pas de réponse."""
    send(ser, CMD_GET_STATUS)
    r = recv(ser, 0.4, expect_resp=RESP_STATUS)
    if not r or len(r[1]) < 25:
        return None
    d = r[1]
    x, y, th   = struct.unpack_from('<fff', d, 0)
    vx, vy, wz = struct.unpack_from('<fff', d, 12)
    state       = d[24]
    return dict(x=x, y=y, th=th, vx=vx, vy=vy, wz=wz,
                state=state, name=STATE_NAMES.get(state, "?"))

def fmt_status(s):
    sc = GRN if s['name'] == 'MOVING' else (
         YLW if s['name'] in ('DONE','SETTLING','IDLE') else
         RED if s['name'] in ('ESTOP','ERROR') else WHT)
    return (f"pos=({s['x']:+.3f},{s['y']:+.3f},{s['th']:+.3f}rad)  "
            f"vel=({s['vx']:+.3f},{s['vy']:+.3f},{s['wz']:+.3f})  "
            f"etat={sc}{s['name']}{RST}")

def stop_burst(ser, n=5):
    """
    Envoie CMD_STOP plusieurs fois pour maximiser la chance qu'au moins
    une trame complète passe entre deux cycles TIM6 du STM32.
    Vide le buffer RX entre chaque tentative.
    """
    cmd_log(f"STOP x{n} (rafale pour franchir la fenetre TIM6)")
    ack_received = False
    for i in range(n):
        ser.reset_input_buffer()
        send(ser, CMD_STOP)
        r = recv(ser, 0.25, expect_resp=RESP_ACK)
        if r and not ack_received:
            ok(f"STOP ACK (tentative {i+1}/{n})")
            ack_received = True
        time.sleep(0.10)
    if not ack_received:
        warn(f"Aucune des {n} tentatives STOP n'a recu d'ACK")
    return ack_received

# ─────────────────────────────────────────────────────────────
# BOUCLE D'UN TEST — silencieuse pendant le mouvement
# ─────────────────────────────────────────────────────────────

def run_test(ser, t, duration):
    """
    1. RESET_ODOM (attend ACK)
    2. MOVE_TO    (attend ACK)
    3. SILENCE TOTAL pendant `duration` s (decompte local Python uniquement)
    4. STOP en rafale pour bien etre recu
    """
    info("Reset odometrie + sortie ESTOP")
    reset_ok = do_reset(ser)
    time.sleep(0.15)

    move_ok = do_move(ser, t['x'], t['y'], t['th'])
    if not move_ok:
        warn("MOVE_TO non acquitte. Possible : STM32 bloque par overrun precedent.")
        warn("On attend tout de meme la duree, puis on tente STOP en rafale.")

    print(f"  {MAG}>> Observe : {t['expect']}{RST}")
    print(f"  {DIM}(silence radio --- aucune trame envoyee pendant le mouvement){RST}")

    t_start = time.time()
    last_disp = -1
    while True:
        elapsed = time.time() - t_start
        remain  = duration - elapsed
        if remain <= 0:
            break
        # Affichage décompte uniquement, AUCUNE écriture sur le port série
        sec_int = int(remain) + 1
        if sec_int != last_disp:
            print(f"\r  {DIM}[{elapsed:5.1f}s | reste {sec_int:3d}s]{RST}  "
                  f"{CYN}mouvement en cours...{RST}        ",
                  end="", flush=True)
            last_disp = sec_int
        time.sleep(0.1)

    print(f"\r  {DIM}[{duration:5.1f}s | reste   0s]{RST}  "
          f"{GRN}fin du mouvement                          {RST}")

    # STOP en rafale
    stop_burst(ser, n=5)

    # Petit GET_STATUS de vérification de responsivité
    time.sleep(0.2)
    s = do_status_quick(ser)
    if s:
        ok(f"STM32 responsif : {fmt_status(s)}")
    else:
        warn("STM32 ne repond pas au GET_STATUS apres STOP --- probable overrun firmware.")
        warn("Le test suivant risque d'echouer. Solution : reset hardware du STM32.")

# ─────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────

_ser_global = None

def sigint_handler(signum, frame):
    print()
    warn("Ctrl+C recu --- STOP x5 et sortie propre")
    if _ser_global is not None:
        try:
            stop_burst(_ser_global, n=5)
            _ser_global.close()
        except Exception:
            pass
    sys.exit(0)

def main():
    global _ser_global

    parser = argparse.ArgumentParser(
        description="Test automatique moteurs mecanum (version silencieuse)"
    )
    parser.add_argument("port", nargs="?", default="/dev/serial0",
                        help="Port serie (defaut: /dev/serial0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (defaut: 115200)")
    parser.add_argument("--dist", type=float, default=0.10,
                        help="Distance par axe en metres (defaut: 0.10)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Duree de chaque test en secondes (defaut: 10)")
    parser.add_argument("--pause", type=float, default=1.0,
                        help="Pause entre 2 tests en secondes (defaut: 1.0)")
    args = parser.parse_args()

    tests       = make_tests(args.dist)
    total_time  = TOTAL * (args.duration + args.pause + 1.5)

    print()
    sep()
    print(f"{BOLD}{CYN}  TEST MOTEURS MECANUM --- MODE AUTOMATIQUE SILENCIEUX{RST}")
    print(f"{DIM}  Raspberry Pi -> USART2 -> STM32 -> UART4 -> Sabertooth{RST}")
    sep()
    info(f"Port      : {args.port}  @{args.baud} baud")
    info(f"Distance  : {args.dist} m / axe  (diagonale ~ {args.dist*math.sqrt(2):.3f} m)")
    info(f"Duree     : {args.duration} s par test  x  {TOTAL} tests  ~  {total_time:.0f} s total")
    info("Strategie : aucune trame pendant le mouvement (evite overrun firmware)")
    info("Ctrl+C pour interrompre et arreter les moteurs.")
    sep()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        err(f"Impossible d'ouvrir {args.port} : {e}")
        sys.exit(1)

    _ser_global = ser
    signal.signal(signal.SIGINT, sigint_handler)

    time.sleep(0.5)
    ser.reset_input_buffer()

    info("Statut initial :")
    s = do_status_quick(ser)
    if s:
        ok(fmt_status(s))
        if s['name'] == 'ESTOP':
            warn("Robot en ESTOP --- un RESET sera fait au debut de chaque test")
    else:
        warn("Pas de reponse au GET_STATUS --- STM32 allume ? cable OK ?")
        warn("Tu peux quand meme lancer les tests, on verra ce qui passe.")

    sep()
    info("Demarrage dans 3 secondes...")
    time.sleep(3.0)

    t_run_start = time.time()
    for idx, t in enumerate(tests):
        sep()
        cat_color = YLW if t['cat'] == 'diag' else (BLU if t['cat'] == 'rot' else GRN)
        print(f"\n  {BOLD}{cat_color}|-- [{idx+1}/{TOTAL}]  TEST {t['id']} --- {t['label']}{RST}")
        print(f"  {BOLD}{cat_color}|{RST}  {WHT}Paire active : {t['pair']}{RST}")
        print(f"  {BOLD}{cat_color}+--{RST}")

        run_test(ser, t, args.duration)
        time.sleep(args.pause)

    sep()
    ok(f"Sequence complete --- {TOTAL} tests effectues en {time.time()-t_run_start:.1f}s")
    stop_burst(ser, n=3)
    ser.close()
    ok("Port ferme. Fin.")
    print()


if __name__ == "__main__":
    main()
