#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
test_moteurs_interactif.py
==========================
Test interactif moteurs mecanum — Raspberry Pi → STM32 → Sabertooth

Pourquoi les tests 1-4 sont des PAIRES en diagonale et non des roues isolées :

  Cinématique mecanum (cf. mecanum.c) :
      FL = (vx - vy - lxy·wz) / r
      FR = (vx + vy + lxy·wz) / r
      RL = (vx + vy - lxy·wz) / r
      RR = (vx - vy + lxy·wz) / r

  C'est une matrice 4×3 → impossible d'isoler une seule roue via (vx,vy,wz).
  En revanche on peut isoler les PAIRES diagonales :

    • FL+RR seules tournent quand   vy = -vx, wz = 0   (cible x = +d, y = -d, θ=0)
        → FL = 2vx, RR = 2vx, FR = RL = 0
    • FR+RL seules tournent quand   vy = +vx, wz = 0   (cible x = +d, y = +d, θ=0)
        → FR = 2vx, RL = 2vx, FL = RR = 0

  C'est le meilleur diagnostic possible sans modifier le firmware STM32 :
  si une seule des deux roues attendues tourne dans un test diagonal, on a
  identifié la roue défaillante.

Séquence :
  1.  Diagonale FL+RR avant     (FL et RR tournent en avant, FR et RL fixes)
  2.  Diagonale FL+RR arrière   (mêmes roues, sens inverse — teste la marche AR)
  3.  Diagonale FR+RL avant
  4.  Diagonale FR+RL arrière
  5.  Déplacement AVANT         (les 4 roues même sens)
  6.  Déplacement ARRIÈRE
  7.  Déplacement GAUCHE        (latéral)
  8.  Déplacement DROITE        (latéral)
  9.  ROTATION gauche           (anti-horaire)
  10. ROTATION droite           (horaire)

Contrôles :
  ENTRÉE  → passer au test suivant
  r       → rejouer le test courant
  s       → STOP d'urgence immédiat
  q       → quitter

Usage :
  python3 test_moteurs_interactif.py [port] [--dist 0.20]
  python3 test_moteurs_interactif.py /dev/serial0
  python3 test_moteurs_interactif.py /dev/ttyACM0 --dist 0.15

Prérequis :
  pip install pyserial
"""

import argparse
import math
import serial
import struct
import sys
import time
import termios
import tty
import select
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

# ─────────────────────────────────────────────────────────────
# LOGS
# ─────────────────────────────────────────────────────────────

def ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def log(symbol, color, msg):
    print(f"{DIM}[{ts()}]{RST} {color}{symbol}  {msg}{RST}", flush=True)

def ok(msg):    log("✓", GRN, msg)
def err(msg):   log("✗", RED, msg)
def warn(msg):  log("⚠", YLW, msg)
def info(msg):  log("·", CYN, msg)
def cmd_log(msg): log("→", BLU, msg)
def sep():
    print(f"{DIM}{'─' * 62}{RST}", flush=True)

# ─────────────────────────────────────────────────────────────
# DÉFINITION DES TESTS
# ─────────────────────────────────────────────────────────────

def make_tests(d):
    """d = distance de test en mètres (par axe, donc déplacement total ~d·√2 pour les diagonales)."""
    ang = math.radians(25.0)  # angle de rotation pure (rad)

    return [
        # ───── PAIRES DIAGONALES (les seules isolables avec MOVE_TO) ─────
        dict(id=1,  cat="diag", label="DIAGONALE FL + RR — avant",
             desc="Seules FL (avant-gauche) et RR (arrière-droite) doivent tourner, vers l'avant.",
             note="FR et RL doivent rester strictement immobiles. Sinon : câblage Sabertooth ou polarité moteur.",
             x= d,   y=-d,  th=0.0),

        dict(id=2,  cat="diag", label="DIAGONALE FL + RR — arrière",
             desc="Mêmes roues qu'au test 1, sens inverse. Teste la commande arrière des Sabertooth.",
             note="Si une roue tourne dans le bon sens à l'avant mais pas à l'arrière → commande 1/5 Sabertooth.",
             x=-d,   y= d,  th=0.0),

        dict(id=3,  cat="diag", label="DIAGONALE FR + RL — avant",
             desc="Seules FR (avant-droite) et RL (arrière-gauche) doivent tourner, vers l'avant.",
             note="FL et RR doivent rester strictement immobiles.",
             x= d,   y= d,  th=0.0),

        dict(id=4,  cat="diag", label="DIAGONALE FR + RL — arrière",
             desc="Mêmes roues qu'au test 3, sens inverse.",
             note="",
             x=-d,   y=-d,  th=0.0),

        # ───── DÉPLACEMENTS COMPLETS ─────
        dict(id=5,  cat="move", label="AVANT",
             desc="Les 4 roues tournent dans le même sens. Robot avance.",
             note="Si le robot dévie au lieu d'aller droit → une roue est plus lente / inversée.",
             x= d,   y=0.0,  th=0.0),

        dict(id=6,  cat="move", label="ARRIÈRE",
             desc="Les 4 roues tournent en sens inverse. Robot recule.",
             note="",
             x=-d,   y=0.0,  th=0.0),

        dict(id=7,  cat="move", label="LATÉRAL  (y > 0)",
             desc="FL+RR dans un sens, FR+RL dans l'autre. Robot glisse latéralement.",
             note="Le sens GAUCHE/DROITE dépend de l'orientation des galets mecanum (X-config). À vérifier visuellement.",
             x=0.0,  y= d,   th=0.0),

        dict(id=8,  cat="move", label="LATÉRAL  (y < 0)",
             desc="Sens latéral inverse du test 7.",
             note="",
             x=0.0,  y=-d,   th=0.0),

        # ───── ROTATIONS PURES ─────
        dict(id=9,  cat="rot",  label="ROTATION  (θ > 0, anti-horaire)",
             desc="FL+RL reculent, FR+RR avancent. Robot pivote sur place.",
             note="Vérifier que le pivotement se fait bien autour du centre (pas une roue qui patine).",
             x=0.0,  y=0.0,  th= ang),

        dict(id=10, cat="rot",  label="ROTATION  (θ < 0, horaire)",
             desc="Sens inverse du test 9.",
             note="",
             x=0.0,  y=0.0,  th=-ang),
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

def recv(ser, timeout=1.0):
    ser.timeout = timeout
    deadline = time.time() + timeout
    while time.time() < deadline:
        b = ser.read(1)
        if not b or b[0] != PROTO_START:
            continue
        hdr = ser.read(2)
        if len(hdr) != 2:
            return None
        resp, ln = hdr[0], hdr[1]
        rest = ser.read(ln + 1)
        if len(rest) != ln + 1:
            return None
        data, crc_rx = rest[:ln], rest[ln]
        if crc_rx != crc(resp, data):
            warn(f"CRC incorrect (reçu {crc_rx:02X}, attendu {crc(resp,data):02X})")
            return None
        return resp, data
    return None

def do_stop(ser):
    frame = send(ser, CMD_STOP)
    cmd_log(f"STOP  [{frame.hex(' ')}]")
    r = recv(ser, 0.5)
    if r and r[0] == RESP_ACK:
        ok("STOP ACK")
    else:
        warn("STOP sans réponse")

def do_reset(ser):
    frame = send(ser, CMD_RESET_ODOM)
    cmd_log(f"RESET_ODOM  [{frame.hex(' ')}]")
    r = recv(ser, 0.5)
    if r and r[0] == RESP_ACK:
        ok("RESET_ODOM ACK")
    else:
        warn("RESET_ODOM sans réponse")

def do_move(ser, x, y, th_rad):
    payload = struct.pack('<fff', x, y, th_rad)
    frame = send(ser, CMD_MOVE_TO, payload)
    cmd_log(f"MOVE_TO  x={x:+.3f} m  y={y:+.3f} m  θ={math.degrees(th_rad):+.1f}°  [{frame.hex(' ')}]")
    r = recv(ser, 1.0)
    if r is None:
        err("Aucune réponse au MOVE_TO")
        return False
    code, data = r
    if code == RESP_ACK:
        ok("MOVE_TO ACK")
        return True
    if code == RESP_ERROR:
        e = ERR_NAMES.get(data[0] if data else 0, "?")
        err(f"MOVE_TO refusé : {e}")
    else:
        warn(f"Réponse inattendue : 0x{code:02X}")
    return False

def do_status(ser):
    send(ser, CMD_GET_STATUS)
    r = recv(ser, 1.0)
    if not r or r[0] != RESP_STATUS or len(r[1]) < 25:
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
# LECTURE CLAVIER NON BLOQUANTE
# ─────────────────────────────────────────────────────────────

def key_available():
    return bool(select.select([sys.stdin], [], [], 0)[0])

def read_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def wait_key_blocking():
    """Attend une touche, mode raw."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

# ─────────────────────────────────────────────────────────────
# BOUCLE D'UN TEST
# ─────────────────────────────────────────────────────────────

def run_test(ser, t):
    """
    Lance un test et surveille l'état en temps réel.
    Retourne : 'next' | 'replay' | 'stop' | 'quit'
    """
    info("Reset odométrie avant le test…")
    do_reset(ser)
    time.sleep(0.15)

    sent = do_move(ser, t['x'], t['y'], t['th'])
    if not sent:
        info("Appuie sur ENTRÉE pour passer au suivant, r=rejouer, q=quitter")

    info("Surveillance — ENTRÉE=suivant  r=rejouer  s=stop  q=quitter")

    t_start    = time.time()
    last_poll  = 0.0
    last_line  = ""
    done_shown = False

    while True:
        now = time.time()

        # ── poll statut ──────────────────────────────────────
        if now - last_poll >= 0.25:
            s = do_status(ser)
            if s:
                line = f"\r  {DIM}[{now-t_start:5.1f}s]{RST}  {fmt_status(s)}"
                pad = max(0, len(last_line) - len(line))
                print(line + " " * pad, end="", flush=True)
                last_line = line

                if s['name'] in ('DONE', 'IDLE') and now - t_start > 0.8 and not done_shown:
                    print()
                    ok(f"Mouvement terminé  ({now-t_start:.1f} s)")
                    info("ENTRÉE pour le test suivant, r=rejouer")
                    done_shown = True
                    last_line = ""
                elif s['name'] == 'ESTOP':
                    print()
                    err("ESTOP actif — vérifie l'ARU (PB14)")
                    return 'stop'
                elif s['name'] == 'ERROR':
                    print()
                    err("Erreur STM32 — hors grille ou autre")
                    return 'stop'
            last_poll = now

        # ── clavier ─────────────────────────────────────────
        if key_available():
            ch = read_key()
            print()
            if ch in ('\r', '\n', ' '):
                return 'next'
            elif ch == 'r':
                return 'replay'
            elif ch == 's':
                return 'stop'
            elif ch == 'q':
                return 'quit'
            # toute autre touche → ignoré, on continue

        time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Test interactif moteurs mecanum — Raspberry Pi → STM32"
    )
    parser.add_argument("port", nargs="?", default="/dev/serial0",
                        help="Port série (défaut: /dev/serial0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (défaut: 115200)")
    parser.add_argument("--dist", type=float, default=0.20,
                        help="Distance de test par axe en mètres (défaut: 0.20)")
    args = parser.parse_args()

    tests = make_tests(args.dist)

    # ── bannière ─────────────────────────────────────────────
    print()
    sep()
    print(f"{BOLD}{CYN}  TEST MOTEURS MECANUM — INTERACTIF{RST}")
    print(f"{DIM}  Raspberry Pi → USART2 → STM32 → UART4 → Sabertooth{RST}")
    sep()
    info(f"Port     : {args.port}  @{args.baud} baud")
    info(f"Distance : {args.dist} m par axe (diagonale ≈ {args.dist*math.sqrt(2):.3f} m)")
    info(f"Tests    : {TOTAL} séquences  (4 paires diagonales + 6 mouvements)")
    sep()
    print(f"  {WHT}ENTRÉE{RST}  → test suivant")
    print(f"  {WHT}r{RST}       → rejouer le test courant")
    print(f"  {WHT}s{RST}       → STOP d'urgence")
    print(f"  {WHT}q{RST}       → quitter")
    sep()
    warn("Rappel : MOVE_TO est un asservissement POSITION → la vitesse")
    warn("décroît près de la cible. Pour des tests à vitesse constante,")
    warn("il faudrait ajouter une CMD_SET_VELOCITY côté firmware.")
    sep()

    # ── ouverture port ───────────────────────────────────────
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        err(f"Impossible d'ouvrir {args.port} : {e}")
        sys.exit(1)

    time.sleep(0.5)
    ser.reset_input_buffer()

    # ── statut initial ────────────────────────────────────────
    info("Statut initial du robot :")
    s = do_status(ser)
    if s:
        ok(fmt_status(s))
        if s['name'] == 'ESTOP':
            err("Robot en ESTOP au démarrage — vérifie l'ARU avant de continuer")
    else:
        warn("Aucune réponse au GET_STATUS — STM32 allumé et flashé ?")

    sep()
    info("Appuie sur ENTRÉE pour démarrer le test 1  (q pour quitter)…")

    ch = wait_key_blocking()
    if ch == 'q':
        info("Abandon.")
        ser.close()
        return

    # ── boucle de tests ──────────────────────────────────────
    idx = 0
    done_ids = set()

    while idx < TOTAL:
        t = tests[idx]

        sep()
        cat_color = YLW if t['cat'] == 'diag' else (BLU if t['cat'] == 'rot' else GRN)
        print(f"\n  {BOLD}{cat_color}[{idx+1}/{TOTAL}]  TEST {t['id']} — {t['label']}{RST}")
        print(f"  {DIM}{t['desc']}{RST}")
        if t['note']:
            print(f"  {YLW}  ↳ {t['note']}{RST}")
        print()

        action = run_test(ser, t)

        # ── stop systématique entre tests ────────────────────
        do_stop(ser)
        time.sleep(0.3)

        if action == 'quit':
            info("Arrêt demandé par l'utilisateur.")
            break
        elif action == 'stop':
            warn("STOP — robot immobilisé.")
            warn("Appuie sur ENTRÉE pour continuer vers le test suivant (q pour quitter)…")
            ch = wait_key_blocking()
            if ch == 'q':
                break
            done_ids.add(t['id'])
            idx += 1
        elif action == 'replay':
            info("Rejouer le même test…")
            time.sleep(0.2)
            # idx ne bouge pas
        else:  # 'next' ou fin naturelle
            done_ids.add(t['id'])
            idx += 1

    # ── bilan ────────────────────────────────────────────────
    sep()
    ok(f"Séquence terminée — {len(done_ids)}/{TOTAL} tests effectués")
    do_stop(ser)
    ser.close()
    ok("Port fermé. Fin.")
    print()


if __name__ == "__main__":
    main()
