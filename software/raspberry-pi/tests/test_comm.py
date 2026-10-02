#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
test_comm.py - sequence de mouvements basiques Pi -> STM32 -> Sabertooth

Sequence par defaut (chaque mouvement dure --duration secondes) :
  1. AVANT      vx = +--vx,   vy = 0,        wz = 0
  2. ARRIERE    vx = ---vx,   vy = 0,        wz = 0
  3. ROT GAUCHE vx = 0,       vy = 0,        wz = +--wz
  4. ROT DROITE vx = 0,       vy = 0,        wz = ---wz

Un STOP est envoye entre chaque mouvement (--pause secondes).

Protocole utilise :
  CMD_SET_VELOCITY (0x07)  payload = vx, vy, wz (3 floats little-endian)
  CMD_STOP         (0x02)
  CMD_GET_STATUS   (0x04)

Le firmware ne fait plus de PID, plus d'odometrie, plus de grille, plus d'ARU.
Il prend la consigne, la passe a Mecanum_SetVelocity et la re-envoie 100 fois
par seconde au Sabertooth.

Usage :
  python3 test_comm.py [port] [--vx 0.20] [--wz 0.80] [--duration 3]
  python3 test_comm.py /dev/serial0
  python3 test_comm.py /dev/serial0 --only avant
  python3 test_comm.py /dev/serial0 --only rot+ --duration 5

Filtres possibles avec --only :
  avant       arriere       rot+ (rotation gauche)       rot- (rotation droite)
  lateral+    lateral-      all (defaut)
"""

import argparse
import serial
import struct
import sys
import time
from datetime import datetime

# Protocole
PROTO_START      = 0xAA
CMD_STOP         = 0x02
CMD_GET_STATUS   = 0x04
CMD_SET_VELOCITY = 0x07
RESP_ACK         = 0xA1
RESP_STATUS      = 0xA3
RESP_ERROR       = 0xA4

# Couleurs
RST  = "\033[0m"
BOLD = "\033[1m"
DIM  = "\033[2m"
GRN  = "\033[92m"
RED  = "\033[91m"
YLW  = "\033[93m"
CYN  = "\033[96m"
BLU  = "\033[94m"
MAG  = "\033[95m"


def ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def log(symbol, color, msg):
    print(f"{DIM}[{ts()}]{RST} {color}{symbol}  {msg}{RST}", flush=True)

def ok(m):    log("v", GRN, m)
def err(m):   log("x", RED, m)
def warn(m):  log("!", YLW, m)
def info(m):  log(".", CYN, m)
def tx(m):    log(">", BLU, m)
def sep():
    print(f"{DIM}{'-' * 64}{RST}", flush=True)


def crc(cmd, payload):
    c = cmd ^ len(payload)
    for b in payload:
        c ^= b
    return c & 0xFF

def frame(cmd, payload=b''):
    return bytes([PROTO_START, cmd, len(payload)]) + payload + bytes([crc(cmd, payload)])

def send(ser, cmd, payload=b''):
    f = frame(cmd, payload)
    ser.write(f)
    ser.flush()
    return f

def recv(ser, timeout=1.0):
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
            warn(f"CRC incorrect (recu {crc_rx:02X}, attendu {crc(resp,data):02X})")
            continue
        return resp, data
    return None


def cmd_status(ser, quiet=False):
    f = send(ser, CMD_GET_STATUS)
    if not quiet:
        tx(f"GET_STATUS  [{f.hex(' ')}]")
    r = recv(ser, 0.5)
    if r is None:
        warn("Pas de reponse au GET_STATUS")
        return None
    code, data = r
    if code == RESP_STATUS and len(data) >= 12:
        vx, vy, wz = struct.unpack_from('<fff', data, 0)
        if not quiet:
            ok(f"STATUS  consigne courante : vx={vx:+.3f}  vy={vy:+.3f}  wz={wz:+.3f}")
        return vx, vy, wz
    warn(f"Reponse inattendue : 0x{code:02X}")
    return None

def cmd_set_velocity(ser, vx, vy, wz):
    payload = struct.pack('<fff', vx, vy, wz)
    f = send(ser, CMD_SET_VELOCITY, payload)
    tx(f"SET_VELOCITY  vx={vx:+.3f}  vy={vy:+.3f}  wz={wz:+.3f}  [{f.hex(' ')}]")
    r = recv(ser, 0.5)
    if r and r[0] == RESP_ACK:
        ok("ACK")
        return True
    if r and r[0] == RESP_ERROR:
        err(f"ERROR code 0x{r[1][0]:02X}")
    else:
        warn("Pas de reponse au SET_VELOCITY")
    return False

def cmd_stop(ser):
    f = send(ser, CMD_STOP)
    tx(f"STOP  [{f.hex(' ')}]")
    r = recv(ser, 0.5)
    if r and r[0] == RESP_ACK:
        ok("STOP ACK")
        return True
    warn("Pas de reponse au STOP")
    return False


def do_move(ser, name, expect, vx, vy, wz, duration):
    """Joue un mouvement : SET_VELOCITY, attend, STOP."""
    sep()
    print(f"  {BOLD}{MAG}{name}{RST}")
    print(f"  {DIM}{expect}{RST}")
    print()

    if not cmd_set_velocity(ser, vx, vy, wz):
        err("Commande refusee, on saute ce mouvement.")
        return False

    # Decompte (silence radio, on n'envoie rien pendant le mouvement)
    for remaining in range(int(duration), 0, -1):
        print(f"  {DIM}reste {remaining} s...{RST}", end="\r", flush=True)
        time.sleep(1.0)
    print(" " * 30, end="\r")

    cmd_stop(ser)
    return True


# Catalogue des mouvements (key, label, vx_factor, vy_factor, wz_factor)
MOVES = {
    "avant":    ("AVANT      (vx > 0)",  "les 4 roues tournent vers l'avant",                +1, 0,  0),
    "arriere":  ("ARRIERE    (vx < 0)",  "les 4 roues tournent vers l'arriere",              -1, 0,  0),
    "rot+":     ("ROTATION + (wz > 0)",  "robot pivote sur lui-meme dans un sens",            0, 0, +1),
    "rot-":     ("ROTATION - (wz < 0)",  "robot pivote sur lui-meme dans l'autre sens",       0, 0, -1),
    "lateral+": ("LATERAL +  (vy > 0)",  "robot glisse lateralement (FL/RR vs FR/RL)",        0, +1, 0),
    "lateral-": ("LATERAL -  (vy < 0)",  "robot glisse lateralement (sens inverse)",          0, -1, 0),
}

DEFAULT_SEQUENCE = ["avant", "arriere", "rot+", "rot-"]


def main():
    parser = argparse.ArgumentParser(description="Sequence de mouvements basiques (mode minimal)")
    parser.add_argument("port", nargs="?", default="/dev/serial0")
    parser.add_argument("--baud",     type=int,   default=115200)
    parser.add_argument("--vx",       type=float, default=0.20,
                        help="Amplitude vitesse avant/arriere en m/s (defaut 0.20)")
    parser.add_argument("--vy",       type=float, default=0.20,
                        help="Amplitude vitesse laterale en m/s (defaut 0.20)")
    parser.add_argument("--wz",       type=float, default=0.80,
                        help="Amplitude vitesse rotation en rad/s (defaut 0.80)")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Duree de chaque mouvement en secondes (defaut 3)")
    parser.add_argument("--pause",    type=float, default=1.0,
                        help="Pause entre 2 mouvements en secondes (defaut 1)")
    parser.add_argument("--only",     type=str,   default="all",
                        help=f"Filtre : {'/'.join(MOVES.keys())}/all (defaut: all = avant+arriere+rot+/rot-)")
    args = parser.parse_args()

    # Selection des mouvements
    if args.only == "all":
        sequence = DEFAULT_SEQUENCE
    elif args.only == "lateral":
        sequence = ["lateral+", "lateral-"]
    elif args.only in MOVES:
        sequence = [args.only]
    else:
        err(f"--only inconnu : {args.only}")
        err(f"Valeurs valides : {', '.join(list(MOVES.keys()) + ['all', 'lateral'])}")
        sys.exit(1)

    print()
    sep()
    print(f"  {BOLD}{CYN}TEST COMM + MOUVEMENTS BASIQUES{RST}")
    sep()
    info(f"Port      : {args.port}  @{args.baud} baud")
    info(f"vx/vy     : {args.vx} / {args.vy} m/s   wz : {args.wz} rad/s")
    info(f"Duree     : {args.duration} s par mouvement, pause {args.pause} s entre")
    info(f"Sequence  : {' -> '.join(sequence)}")
    sep()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        err(f"Impossible d'ouvrir {args.port} : {e}")
        sys.exit(1)

    time.sleep(0.3)
    ser.reset_input_buffer()

    # Ping initial
    info("Ping STM32 (status initial)...")
    if cmd_status(ser) is None:
        err("Le STM32 ne repond pas. Firmware flash ? cable USB ?")
        ser.close()
        sys.exit(2)

    info("Demarrage dans 3 secondes... (Ctrl+C pour annuler)")
    time.sleep(3.0)

    # Sequence
    t_run = time.time()
    for key in sequence:
        label, expect, fx, fy, fw = MOVES[key]
        vx = args.vx * fx
        vy = args.vy * fy
        wz = args.wz * fw
        do_move(ser, label, expect, vx, vy, wz, args.duration)
        time.sleep(args.pause)

    # Verification finale
    sep()
    info("Verification : la consigne doit etre (0, 0, 0)")
    cmd_status(ser)
    ser.close()

    sep()
    ok(f"Sequence terminee en {time.time()-t_run:.1f} s.")
    print()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        warn("Ctrl+C - tentative d'arret propre.")
        # Best effort : on tente un STOP sur le port par defaut
        try:
            s = serial.Serial("/dev/serial0", 115200, timeout=0.5)
            s.write(frame(CMD_STOP))
            s.close()
        except Exception:
            pass
        sys.exit(130)
