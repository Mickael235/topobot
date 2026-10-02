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

  Si une seule des deux roues attendues tourne dans un test diagonal → roue défaillante identifiée.

Séquence (ordre choisi pour ALTERNER les paires → transitions clairement visibles) :
  1.  Diagonale FL+RR avant     (FL+RR tournent,  FR+RL FIXES)
  2.  Diagonale FR+RL avant     (FR+RL tournent,  FL+RR FIXES)  ← paire différente
  3.  Diagonale FL+RR arrière   (FL+RR tournent dans l'autre sens)
  4.  Diagonale FR+RL arrière   (FR+RL tournent dans l'autre sens)
  5.  Déplacement AVANT
  6.  Déplacement ARRIÈRE
  7.  Déplacement LATÉRAL +y
  8.  Déplacement LATÉRAL -y
  9.  ROTATION θ>0
  10. ROTATION θ<0

Contrôles :
  ENTRÉE / ESPACE → passer au test suivant
  r               → rejouer le test courant
  s               → STOP d'urgence immédiat
  q               → quitter

Usage :
  python3 test_moteurs_interactif.py [port] [--dist 0.10] [--auto 6]
  python3 test_moteurs_interactif.py /dev/serial0
  python3 test_moteurs_interactif.py /dev/ttyACM0 --dist 0.08 --auto 5

Options utiles :
  --dist 0.10   amplitude de cible par axe (m). Plus petit = test plus court.
                Pour les paires diagonales, déplacement total = dist·√2.
  --auto 6      passage automatique au test suivant après N secondes même si
                le robot n'a pas atteint sa cible (utile si moteurs poussifs).
                0 = pas d'auto-advance (passage uniquement par ENTRÉE).

Prérequis : pip install pyserial
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
MAG  = "\033[95m"

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
# DÉFINITION DES TESTS — ordre alterné pour visibilité maximale
# ─────────────────────────────────────────────────────────────

def make_tests(d):
    """d = distance par axe (m). Pour les diagonales, déplacement total = d·√2."""
    ang = math.radians(25.0)  # rotation pure (rad)

    return [
        # ───── 1 & 2 : AVANT — paires différentes ─────
        dict(id=1, cat="diag", pair="FL+RR", label="DIAGONALE FL + RR — AVANT",
             desc="Seules FL (avant-gauche) et RR (arrière-droite) doivent tourner.",
             expect="FL+RR tournent vers l'AVANT  •  FR et RL strictement IMMOBILES",
             note="Si FR ou RL bouge → câblage / adresse Sabertooth à vérifier.",
             x= d,   y=-d,  th=0.0),

        dict(id=2, cat="diag", pair="FR+RL", label="DIAGONALE FR + RL — AVANT",
             desc="Seules FR (avant-droite) et RL (arrière-gauche) doivent tourner.",
             expect="FR+RL tournent vers l'AVANT  •  FL et RR strictement IMMOBILES",
             note="Paire opposée du test 1 — la transition doit être VISIBLE.",
             x= d,   y= d,  th=0.0),

        # ───── 3 & 4 : ARRIÈRE — alternance paires ─────
        dict(id=3, cat="diag", pair="FL+RR", label="DIAGONALE FL + RR — ARRIÈRE",
             desc="Mêmes roues qu'au test 1, sens INVERSE.",
             expect="FL+RR tournent vers l'ARRIÈRE  •  FR et RL immobiles",
             note="Teste la commande Sabertooth 1 / 5 (marche arrière).",
             x=-d,   y= d,  th=0.0),

        dict(id=4, cat="diag", pair="FR+RL", label="DIAGONALE FR + RL — ARRIÈRE",
             desc="Mêmes roues qu'au test 2, sens INVERSE.",
             expect="FR+RL tournent vers l'ARRIÈRE  •  FL et RR immobiles",
             note="",
             x=-d,   y=-d,  th=0.0),

        # ───── 5-10 : déplacements complets ─────
        dict(id=5, cat="move", pair="ALL", label="AVANT",
             desc="Les 4 roues tournent dans le même sens. Robot avance.",
             expect="Les 4 roues tournent vers l'AVANT à vitesse égale",
             note="Si le robot dévie → une roue plus lente ou inversée.",
             x= d,   y=0.0, th=0.0),

        dict(id=6, cat="move", pair="ALL", label="ARRIÈRE",
             desc="Les 4 roues en sens inverse. Robot recule.",
             expect="Les 4 roues tournent vers l'ARRIÈRE",
             note="",
             x=-d,   y=0.0, th=0.0),

        dict(id=7, cat="move", pair="LAT", label="LATÉRAL +y",
             desc="FL+RR dans un sens, FR+RL dans l'autre. Robot glisse latéralement.",
             expect="2 roues en avant, 2 en arrière (opposition diagonale)",
             note="Le sens GAUCHE/DROITE dépend des galets mecanum, à vérifier visuellement.",
             x=0.0,  y= d,  th=0.0),

        dict(id=8, cat="move", pair="LAT", label="LATÉRAL -y",
             desc="Sens latéral inverse du test 7.",
             expect="Opposition diagonale inversée",
             note="",
             x=0.0,  y=-d,  th=0.0),

        dict(id=9, cat="rot",  pair="ROT", label="ROTATION θ>0",
             desc="FL+RL reculent, FR+RR avancent. Pivot sur place.",
             expect="Le robot tourne sur lui-même sans translation notable",
             note="Si une roue patine → vérifier la pression au sol.",
             x=0.0,  y=0.0, th= ang),

        dict(id=10, cat="rot", pair="ROT", label="ROTATION θ<0",
             desc="Sens inverse du test 9.",
             expect="Rotation dans le sens opposé",
             note="",
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
    """
    Lit une trame. Si expect_resp est donné, ignore les trames d'un autre type
    (utile car le STM32 peut envoyer un RESP_DONE non sollicité quand le robot
    finit son mouvement, ce qui pollue les recv suivants).
    """
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
            # Trame valide mais pas celle attendue → on l'ignore et on continue
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
        ok("RESET_ODOM ACK — odométrie et état remis à zéro")
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
        err(f"MOVE_TO REFUSÉ : {e}  (robot peut-être en ESTOP — appuie sur 's' puis ENTRÉE pour reprendre)")
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
# LECTURE CLAVIER
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
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def drain_stdin():
    """Vide tout caractère résiduel dans stdin (ex: '\\n' laissé par un 'r\\n')."""
    while key_available():
        try:
            read_key()
        except Exception:
            break

# ─────────────────────────────────────────────────────────────
# BOUCLE D'UN TEST
# ─────────────────────────────────────────────────────────────

def run_test(ser, t, auto_advance):
    """
    Lance un test, surveille en temps réel.
    auto_advance : passe automatiquement au test suivant après N secondes (0 = désactivé).
    Retourne : 'next' | 'replay' | 'stop' | 'quit'
    """
    info("Reset odométrie + sortie ESTOP…")
    do_reset(ser)
    time.sleep(0.2)

    sent = do_move(ser, t['x'], t['y'], t['th'])
    if not sent:
        warn("La commande n'a pas été acceptée — appuie sur ENTRÉE pour le test suivant")

    print(f"  {MAG}▶ Observe les roues : {t['expect']}{RST}")
    print(f"  {DIM}ENTRÉE=suivant  r=rejouer  s=stop  q=quitter"
          f"{('  (auto-suivant dans ' + str(auto_advance) + 's)') if auto_advance > 0 else ''}{RST}")

    t_start    = time.time()
    last_poll  = 0.0
    last_line  = ""
    done_shown = False

    while True:
        now = time.time()
        elapsed = now - t_start

        # ── auto-advance ────────────────────────────────────
        if auto_advance > 0 and elapsed >= auto_advance:
            print()
            info(f"Auto-passage au test suivant après {auto_advance}s")
            return 'next'

        # ── poll statut ─────────────────────────────────────
        if now - last_poll >= 0.30:
            s = do_status(ser)
            if s:
                line = f"\r  {DIM}[{elapsed:5.1f}s]{RST}  {fmt_status(s)}"
                pad = max(0, len(last_line) - len(line))
                print(line + " " * pad, end="", flush=True)
                last_line = line

                if s['name'] in ('DONE', 'IDLE') and elapsed > 0.8 and not done_shown:
                    print()
                    ok(f"Cible atteinte  ({elapsed:.1f} s) — ENTRÉE pour le suivant")
                    done_shown = True
                    last_line = ""
                elif s['name'] == 'ESTOP':
                    print()
                    err("ESTOP — vérifie l'ARU (PB14)")
                    return 'stop'
                elif s['name'] == 'ERROR':
                    print()
                    err("Erreur STM32 (hors grille ?)")
                    return 'stop'
            last_poll = now

        # ── clavier ─────────────────────────────────────────
        if key_available():
            ch = read_key()
            print()
            if ch in ('\r', '\n', ' '):
                info("→ passage au test suivant")
                return 'next'
            elif ch == 'r':
                info("→ rejouer ce test")
                drain_stdin()
                return 'replay'
            elif ch == 's':
                info("→ STOP")
                return 'stop'
            elif ch == 'q':
                info("→ quitter")
                return 'quit'
            # autre touche → ignoré

        time.sleep(0.04)

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
    parser.add_argument("--dist", type=float, default=0.10,
                        help="Distance par axe en mètres (défaut: 0.10)")
    parser.add_argument("--auto", type=float, default=0,
                        help="Auto-passage après N secondes (0 = désactivé, défaut: 0)")
    args = parser.parse_args()

    tests = make_tests(args.dist)

    # ── bannière ─────────────────────────────────────────────
    print()
    sep()
    print(f"{BOLD}{CYN}  TEST MOTEURS MECANUM — INTERACTIF{RST}")
    print(f"{DIM}  Raspberry Pi → USART2 → STM32 → UART4 → Sabertooth{RST}")
    sep()
    info(f"Port      : {args.port}  @{args.baud} baud")
    info(f"Distance  : {args.dist} m / axe (diagonale ≈ {args.dist*math.sqrt(2):.3f} m)")
    info(f"Tests     : {TOTAL} séquences  (4 diagonales alternées + 6 mouvements)")
    if args.auto > 0:
        info(f"Auto-next : {args.auto} s")
    sep()
    print(f"  {WHT}ENTRÉE / ESPACE{RST} → test suivant")
    print(f"  {WHT}r{RST}              → rejouer le test courant")
    print(f"  {WHT}s{RST}              → STOP d'urgence")
    print(f"  {WHT}q{RST}              → quitter")
    sep()
    warn("MOVE_TO = asservissement POSITION : la vitesse décroît près de la cible.")
    warn("Si les moteurs sont poussifs, utilise --dist plus petit ou --auto pour")
    warn("ne pas rester bloqué dans un test qui n'atteint jamais sa cible.")
    sep()

    # ── ouverture port ───────────────────────────────────────
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        err(f"Impossible d'ouvrir {args.port} : {e}")
        sys.exit(1)

    time.sleep(0.5)
    ser.reset_input_buffer()

    info("Statut initial :")
    s = do_status(ser)
    if s:
        ok(fmt_status(s))
        if s['name'] == 'ESTOP':
            warn("Robot en ESTOP — un RESET sera fait au début du test 1")
    else:
        warn("Pas de réponse au GET_STATUS — STM32 allumé ? câble OK ?")

    sep()
    info("ENTRÉE pour démarrer le test 1  (q = quitter)…")

    drain_stdin()
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
        print(f"\n  {BOLD}{cat_color}┌─ [{idx+1}/{TOTAL}]  TEST {t['id']} — {t['label']}{RST}")
        print(f"  {BOLD}{cat_color}│{RST}  {DIM}{t['desc']}{RST}")
        print(f"  {BOLD}{cat_color}│{RST}  {WHT}Paire active : {t['pair']}{RST}")
        if t['note']:
            print(f"  {BOLD}{cat_color}│{RST}  {YLW}↳ {t['note']}{RST}")
        print(f"  {BOLD}{cat_color}└─{RST}")

        drain_stdin()  # éviter qu'un '\n' résiduel saute immédiatement le test
        action = run_test(ser, t, args.auto)

        do_stop(ser)
        time.sleep(0.3)

        if action == 'quit':
            info("Arrêt demandé par l'utilisateur.")
            break
        elif action == 'stop':
            warn("STOP — robot immobilisé. ENTRÉE pour continuer, q pour quitter…")
            drain_stdin()
            ch = wait_key_blocking()
            if ch == 'q':
                break
            done_ids.add(t['id'])
            idx += 1
        elif action == 'replay':
            info(f"Rejouer le test {t['id']}…")
            time.sleep(0.2)
            # idx ne bouge pas
        else:  # 'next'
            done_ids.add(t['id'])
            idx += 1

    sep()
    ok(f"Séquence terminée — {len(done_ids)}/{TOTAL} tests effectués")
    do_stop(ser)
    ser.close()
    ok("Port fermé. Fin.")
    print()


if __name__ == "__main__":
    main()
