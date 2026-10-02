#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
test_moteurs_auto.py
====================
Test AUTOMATIQUE des moteurs mecanum — sans interaction clavier.

Chaque test dure `--duration` secondes, puis STOP et passage au suivant.
Aucune attente d'entrée utilisateur, juste à observer le robot et la console.

Séquence (ordre alterné pour transitions visibles entre tests successifs) :
  1.  Diagonale FL+RR avant     (FL+RR tournent,  FR+RL FIXES)
  2.  Diagonale FR+RL avant     (FR+RL tournent,  FL+RR FIXES)   ← paire opposée
  3.  Diagonale FL+RR arrière
  4.  Diagonale FR+RL arrière
  5.  Déplacement AVANT
  6.  Déplacement ARRIÈRE
  7.  Déplacement LATÉRAL +y
  8.  Déplacement LATÉRAL -y
  9.  ROTATION θ>0
  10. ROTATION θ<0

Usage :
  python3 test_moteurs_auto.py [port] [--dist 0.10] [--duration 10]
  python3 test_moteurs_auto.py /dev/serial0
  python3 test_moteurs_auto.py /dev/ttyACM0 --dist 0.08 --duration 8

Interruption : Ctrl+C → STOP immédiat et sortie propre.

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

def ok(msg):      log("✓", GRN, msg)
def err(msg):     log("✗", RED, msg)
def warn(msg):    log("⚠", YLW, msg)
def info(msg):    log("·", CYN, msg)
def cmd_log(msg): log("→", BLU, msg)
def sep():
    print(f"{DIM}{'─' * 62}{RST}", flush=True)

# ─────────────────────────────────────────────────────────────
# DÉFINITION DES TESTS
# ─────────────────────────────────────────────────────────────

def make_tests(d):
    """d = distance par axe (m). Pour les diagonales, déplacement total = d·√2."""
    ang = math.radians(25.0)

    return [
        dict(id=1, cat="diag", pair="FL+RR", label="DIAGONALE FL + RR — AVANT",
             expect="FL+RR tournent vers l'AVANT  •  FR et RL strictement IMMOBILES",
             x= d,   y=-d,  th=0.0),

        dict(id=2, cat="diag", pair="FR+RL", label="DIAGONALE FR + RL — AVANT",
             expect="FR+RL tournent vers l'AVANT  •  FL et RR strictement IMMOBILES",
             x= d,   y= d,  th=0.0),

        dict(id=3, cat="diag", pair="FL+RR", label="DIAGONALE FL + RR — ARRIÈRE",
             expect="FL+RR tournent vers l'ARRIÈRE  •  FR et RL immobiles",
             x=-d,   y= d,  th=0.0),

        dict(id=4, cat="diag", pair="FR+RL", label="DIAGONALE FR + RL — ARRIÈRE",
             expect="FR+RL tournent vers l'ARRIÈRE  •  FL et RR immobiles",
             x=-d,   y=-d,  th=0.0),

        dict(id=5, cat="move", pair="ALL", label="AVANT",
             expect="Les 4 roues tournent vers l'AVANT à vitesse égale",
             x= d,   y=0.0, th=0.0),

        dict(id=6, cat="move", pair="ALL", label="ARRIÈRE",
             expect="Les 4 roues tournent vers l'ARRIÈRE",
             x=-d,   y=0.0, th=0.0),

        dict(id=7, cat="move", pair="LAT", label="LATÉRAL +y",
             expect="2 roues en avant, 2 en arrière (opposition diagonale)",
             x=0.0,  y= d,  th=0.0),

        dict(id=8, cat="move", pair="LAT", label="LATÉRAL -y",
             expect="Opposition diagonale inversée",
             x=0.0,  y=-d,  th=0.0),

        dict(id=9, cat="rot",  pair="ROT", label="ROTATION θ>0",
             expect="Le robot pivote sur lui-même (sens 1)",
             x=0.0,  y=0.0, th= ang),

        dict(id=10, cat="rot", pair="ROT", label="ROTATION θ<0",
             expect="Le robot pivote sur lui-même (sens inverse)",
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
    """Lit une trame. Si expect_resp donné, ignore les trames d'un autre type."""
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
            warn(f"CRC incorrect (reçu {crc_rx:02X}, attendu {crc(resp,data):02X})")
            continue
        if expect_resp is not None and resp != expect_resp:
            continue
        return resp, data
    return None

def do_stop(ser):
    frame = send(ser, CMD_STOP)
    cmd_log(f"STOP  [{frame.hex(' ')}]")
    r = recv(ser, 0.5, expect_resp=RESP_ACK)
    if r:
        ok("STOP ACK")
    else:
        warn("STOP sans réponse")

def do_reset(ser):
    frame = send(ser, CMD_RESET_ODOM)
    cmd_log(f"RESET_ODOM  [{frame.hex(' ')}]")
    r = recv(ser, 0.5, expect_resp=RESP_ACK)
    if r:
        ok("RESET_ODOM ACK")
    else:
        warn("RESET_ODOM sans réponse")

def do_move(ser, x, y, th_rad):
    payload = struct.pack('<fff', x, y, th_rad)
    frame = send(ser, CMD_MOVE_TO, payload)
    cmd_log(f"MOVE_TO  x={x:+.3f}m  y={y:+.3f}m  θ={math.degrees(th_rad):+.1f}°  [{frame.hex(' ')}]")
    r = recv(ser, 1.0)
    if r is None:
        err("Aucune réponse au MOVE_TO")
        return False
    code, data = r
    if code == RESP_ACK:
        ok("MOVE_TO ACK — robot en route")
        return True
    if code == RESP_ERROR:
        e = ERR_NAMES.get(data[0] if data else 0, "?")
        err(f"MOVE_TO REFUSÉ : {e}")
    else:
        warn(f"Réponse inattendue : 0x{code:02X}")
    return False

def do_status(ser):
    send(ser, CMD_GET_STATUS)
    r = recv(ser, 0.5, expect_resp=RESP_STATUS)
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
            f"état={sc}{s['name']}{RST}")

# ─────────────────────────────────────────────────────────────
# BOUCLE D'UN TEST (automatique, durée fixe)
# ─────────────────────────────────────────────────────────────

def run_test(ser, t, duration):
    """Lance le test, scrute le statut pendant `duration` secondes, puis retourne."""
    info("Reset odométrie + sortie ESTOP…")
    do_reset(ser)
    time.sleep(0.2)

    sent = do_move(ser, t['x'], t['y'], t['th'])
    if not sent:
        warn(f"Commande refusée — on attend tout de même {duration}s avant le test suivant")

    print(f"  {MAG}▶ Observe : {t['expect']}{RST}")

    t_start   = time.time()
    last_poll = 0.0
    last_line = ""

    while True:
        now     = time.time()
        elapsed = now - t_start
        remain  = duration - elapsed

        if remain <= 0:
            print()
            info(f"Fin du test {t['id']}  ({elapsed:.1f}s écoulées)")
            return

        if now - last_poll >= 0.30:
            s = do_status(ser)
            if s:
                line = (f"\r  {DIM}[{elapsed:5.1f}s | reste {remain:4.1f}s]{RST}  "
                        f"{fmt_status(s)}")
                pad = max(0, len(last_line) - len(line))
                print(line + " " * pad, end="", flush=True)
                last_line = line

                if s['name'] == 'ESTOP':
                    print()
                    err("ESTOP actif — vérifie l'ARU (PB14). On continue le décompte.")
                elif s['name'] == 'ERROR':
                    print()
                    err("Erreur STM32 (hors grille ?). On continue le décompte.")
            last_poll = now

        time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────

# Variable globale pour le handler Ctrl+C
_ser_global = None

def sigint_handler(signum, frame):
    print()
    warn("Ctrl+C reçu — STOP et sortie propre")
    if _ser_global is not None:
        try:
            do_stop(_ser_global)
            _ser_global.close()
        except Exception:
            pass
    sys.exit(0)

def main():
    global _ser_global

    parser = argparse.ArgumentParser(
        description="Test AUTOMATIQUE moteurs mecanum — sans interaction clavier"
    )
    parser.add_argument("port", nargs="?", default="/dev/serial0",
                        help="Port série (défaut: /dev/serial0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (défaut: 115200)")
    parser.add_argument("--dist", type=float, default=0.10,
                        help="Distance par axe en mètres (défaut: 0.10)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Durée de chaque test en secondes (défaut: 10)")
    parser.add_argument("--pause", type=float, default=0.8,
                        help="Pause entre 2 tests en secondes (défaut: 0.8)")
    args = parser.parse_args()

    tests       = make_tests(args.dist)
    total_time  = TOTAL * (args.duration + args.pause)

    # ── bannière ─────────────────────────────────────────────
    print()
    sep()
    print(f"{BOLD}{CYN}  TEST MOTEURS MECANUM — MODE AUTOMATIQUE{RST}")
    print(f"{DIM}  Raspberry Pi → USART2 → STM32 → UART4 → Sabertooth{RST}")
    sep()
    info(f"Port      : {args.port}  @{args.baud} baud")
    info(f"Distance  : {args.dist} m / axe (diagonale ≈ {args.dist*math.sqrt(2):.3f} m)")
    info(f"Durée     : {args.duration} s par test  ×  {TOTAL} tests  ≈  {total_time:.0f} s total")
    info("Aucune interaction clavier — observer juste le robot.")
    info("Ctrl+C pour interrompre et arrêter les moteurs.")
    sep()

    # ── ouverture port ───────────────────────────────────────
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
    s = do_status(ser)
    if s:
        ok(fmt_status(s))
        if s['name'] == 'ESTOP':
            warn("Robot en ESTOP — un RESET sera fait au début de chaque test")
    else:
        warn("Pas de réponse au GET_STATUS — STM32 allumé ? câble OK ?")

    sep()
    info(f"Démarrage dans 3 secondes…")
    time.sleep(3.0)

    # ── boucle de tests ──────────────────────────────────────
    t_run_start = time.time()
    for idx, t in enumerate(tests):
        sep()
        cat_color = YLW if t['cat'] == 'diag' else (BLU if t['cat'] == 'rot' else GRN)
        print(f"\n  {BOLD}{cat_color}┌─ [{idx+1}/{TOTAL}]  TEST {t['id']} — {t['label']}{RST}")
        print(f"  {BOLD}{cat_color}│{RST}  {WHT}Paire active : {t['pair']}{RST}")
        print(f"  {BOLD}{cat_color}└─{RST}")

        run_test(ser, t, args.duration)

        do_stop(ser)
        time.sleep(args.pause)

    # ── bilan ────────────────────────────────────────────────
    sep()
    ok(f"Séquence complète — {TOTAL} tests effectués en {time.time()-t_run_start:.1f}s")
    do_stop(ser)
    ser.close()
    ok("Port fermé. Fin.")
    print()


if __name__ == "__main__":
    main()
