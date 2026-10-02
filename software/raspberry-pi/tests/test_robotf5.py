#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
test_moteurs_v3.py
==================
Test moteurs mecanum — version DIAGNOSTIC, utilise les nouvelles commandes
bas niveau du firmware (CMD_TEST_WHEEL + CMD_RAW_VELOCITY) qui court-circuitent
le PID, la cinematique inverse, l'odometrie et la grille de securite.

Necessite le firmware patche :
  - CMD_TEST_WHEEL   (0x07)  : drive UNE seule roue par son id
  - CMD_RAW_VELOCITY (0x08)  : envoie vx/vy/wz a Mecanum_SetVelocity sans PID
  - HAL_UART_ErrorCallback dans main.c (sinon overrun bloque le STM32)

Sequence :
  1.  Roue FL seule, avant
  2.  Roue FL seule, arriere
  3.  Roue FR seule, avant
  4.  Roue FR seule, arriere
  5.  Roue RL seule, avant
  6.  Roue RL seule, arriere
  7.  Roue RR seule, avant
  8.  Roue RR seule, arriere
  9.  Deplacement AVANT          (raw vx > 0)
  10. Deplacement ARRIERE        (raw vx < 0)
  11. Deplacement LATERAL +y     (raw vy > 0)
  12. Deplacement LATERAL -y     (raw vy < 0)
  13. ROTATION wz > 0            (raw wz > 0)
  14. ROTATION wz < 0            (raw wz < 0)

Chaque test dure --duration secondes. Aucune trame n'est envoyee pendant le
mouvement (evite l'overrun UART). STOP envoye en rafale a la fin.

Usage :
  python3 test_moteurs_v3.py [port] [--norm 0.30] [--duration 6]
  python3 test_moteurs_v3.py /dev/serial0
  python3 test_moteurs_v3.py /dev/ttyACM0 --norm 0.25 --duration 5

Prerequis : pip install pyserial
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
# PROTOCOLE STM32 (firmware patche)
# ─────────────────────────────────────────────────────────────

PROTO_START      = 0xAA
CMD_MOVE_TO      = 0x01
CMD_STOP         = 0x02
CMD_RESET_ODOM   = 0x03
CMD_GET_STATUS   = 0x04
CMD_TEST_WHEEL   = 0x07   # NEW : wheel_id(1B) + norm(float)
CMD_RAW_VELOCITY = 0x08   # NEW : vx(float) + vy(float) + wz(float)

RESP_ACK    = 0xA1
RESP_DONE   = 0xA2
RESP_STATUS = 0xA3
RESP_ERROR  = 0xA4

WHEEL_FL = 0
WHEEL_FR = 1
WHEEL_RL = 2
WHEEL_RR = 3

WHEEL_NAMES = {0: "FL (avant-gauche) ",
               1: "FR (avant-droite) ",
               2: "RL (arriere-gauche)",
               3: "RR (arriere-droite)"}

STATE_NAMES = {0:"IDLE", 1:"MOVING", 2:"SETTLING", 3:"DONE", 4:"ESTOP", 5:"ERROR", 6:"RAW"}
ERR_NAMES   = {0x01:"ERR_CRC", 0x02:"ERR_UNKNOWN_CMD", 0x03:"ERR_BUSY",
               0x04:"ERR_OUT_BOUNDS", 0x05:"ERR_ARU", 0x06:"ERR_AX12"}

# ─────────────────────────────────────────────────────────────
# COULEURS / LOGS
# ─────────────────────────────────────────────────────────────

RST, BOLD, DIM = "\033[0m", "\033[1m", "\033[2m"
GRN, RED, YLW  = "\033[92m", "\033[91m", "\033[93m"
CYN, WHT, BLU  = "\033[96m", "\033[97m", "\033[94m"
MAG            = "\033[95m"

def ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def log(symbol, color, msg):
    print(f"{DIM}[{ts()}]{RST} {color}{symbol}  {msg}{RST}", flush=True)

def ok(m):    log("v", GRN, m)
def err(m):   log("x", RED, m)
def warn(m):  log("!", YLW, m)
def info(m):  log(".", CYN, m)
def cmd_log(m): log(">", BLU, m)
def sep():    print(f"{DIM}{'-' * 64}{RST}", flush=True)

# ─────────────────────────────────────────────────────────────
# DEFINITION DES TESTS
# ─────────────────────────────────────────────────────────────

def make_tests(n):
    """n = normalized speed (norme [0, 1])."""
    return [
        # ===== ROUES INDIVIDUELLES via CMD_TEST_WHEEL =====
        dict(id=1, kind="wheel", wheel=WHEEL_FL, norm=+n,
             label=f"ROUE FL  avant  (norm=+{n:.2f})",
             expect="SEULE la roue FL doit tourner vers l'avant. FR, RL, RR strictement immobiles."),
        dict(id=2, kind="wheel", wheel=WHEEL_FL, norm=-n,
             label=f"ROUE FL  arriere  (norm=-{n:.2f})",
             expect="SEULE FL tourne vers l'arriere."),

        dict(id=3, kind="wheel", wheel=WHEEL_FR, norm=+n,
             label=f"ROUE FR  avant  (norm=+{n:.2f})",
             expect="SEULE FR tourne vers l'avant."),
        dict(id=4, kind="wheel", wheel=WHEEL_FR, norm=-n,
             label=f"ROUE FR  arriere  (norm=-{n:.2f})",
             expect="SEULE FR tourne vers l'arriere."),

        dict(id=5, kind="wheel", wheel=WHEEL_RL, norm=+n,
             label=f"ROUE RL  avant  (norm=+{n:.2f})",
             expect="SEULE RL tourne vers l'avant."),
        dict(id=6, kind="wheel", wheel=WHEEL_RL, norm=-n,
             label=f"ROUE RL  arriere  (norm=-{n:.2f})",
             expect="SEULE RL tourne vers l'arriere."),

        dict(id=7, kind="wheel", wheel=WHEEL_RR, norm=+n,
             label=f"ROUE RR  avant  (norm=+{n:.2f})",
             expect="SEULE RR tourne vers l'avant."),
        dict(id=8, kind="wheel", wheel=WHEEL_RR, norm=-n,
             label=f"ROUE RR  arriere  (norm=-{n:.2f})",
             expect="SEULE RR tourne vers l'arriere."),

        # ===== DEPLACEMENTS COMPLETS via CMD_RAW_VELOCITY =====
        dict(id=9, kind="vel", vx=+n*0.4, vy=0.0, wz=0.0,
             label=f"AVANT  vx=+{n*0.4:.2f} m/s",
             expect="Les 4 roues tournent vers l'avant a vitesse egale. Robot avance."),
        dict(id=10, kind="vel", vx=-n*0.4, vy=0.0, wz=0.0,
             label=f"ARRIERE  vx=-{n*0.4:.2f} m/s",
             expect="Les 4 roues tournent vers l'arriere. Robot recule."),

        dict(id=11, kind="vel", vx=0.0, vy=+n*0.4, wz=0.0,
             label=f"LATERAL +y  vy=+{n*0.4:.2f} m/s",
             expect="Strafe : FL+RR dans un sens, FR+RL dans l'autre."),
        dict(id=12, kind="vel", vx=0.0, vy=-n*0.4, wz=0.0,
             label=f"LATERAL -y  vy=-{n*0.4:.2f} m/s",
             expect="Strafe inverse."),

        dict(id=13, kind="vel", vx=0.0, vy=0.0, wz=+n*1.2,
             label=f"ROTATION wz=+{n*1.2:.2f} rad/s",
             expect="Robot pivote sur lui-meme (sens 1)."),
        dict(id=14, kind="vel", vx=0.0, vy=0.0, wz=-n*1.2,
             label=f"ROTATION wz=-{n*1.2:.2f} rad/s",
             expect="Robot pivote sur lui-meme (sens inverse)."),
    ]

# ─────────────────────────────────────────────────────────────
# PROTOCOLE - ENCODE / DECODE
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

def do_test_wheel(ser, wheel_id, norm):
    payload = struct.pack('<Bf', wheel_id, norm)
    frame = send(ser, CMD_TEST_WHEEL, payload)
    cmd_log(f"TEST_WHEEL  id={wheel_id} ({WHEEL_NAMES[wheel_id].strip()})  norm={norm:+.2f}  [{frame.hex(' ')}]")
    r = recv(ser, 0.8, expect_resp=RESP_ACK)
    if r:
        ok("TEST_WHEEL ACK")
        return True
    warn("TEST_WHEEL sans ACK")
    return False

def do_raw_velocity(ser, vx, vy, wz):
    payload = struct.pack('<fff', vx, vy, wz)
    frame = send(ser, CMD_RAW_VELOCITY, payload)
    cmd_log(f"RAW_VELOCITY  vx={vx:+.3f}  vy={vy:+.3f}  wz={wz:+.3f}  [{frame.hex(' ')}]")
    r = recv(ser, 0.8, expect_resp=RESP_ACK)
    if r:
        ok("RAW_VELOCITY ACK")
        return True
    warn("RAW_VELOCITY sans ACK")
    return False

def do_status_quick(ser):
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
    sc = GRN if s['name'] in ('MOVING','RAW') else (
         YLW if s['name'] in ('DONE','SETTLING','IDLE') else
         RED if s['name'] in ('ESTOP','ERROR') else WHT)
    return (f"pos=({s['x']:+.3f},{s['y']:+.3f},{s['th']:+.3f}rad)  "
            f"etat={sc}{s['name']}{RST}")

def stop_burst(ser, n=5):
    cmd_log(f"STOP x{n} (rafale)")
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
# UN TEST
# ─────────────────────────────────────────────────────────────

def run_test(ser, t, duration):
    """Lance le test, silence pendant duration, STOP burst, status."""
    if t['kind'] == 'wheel':
        sent = do_test_wheel(ser, t['wheel'], t['norm'])
    else:
        sent = do_raw_velocity(ser, t['vx'], t['vy'], t['wz'])

    if not sent:
        warn("Commande non acquittee. On attend tout de meme.")

    print(f"  {MAG}>> Observe : {t['expect']}{RST}")
    print(f"  {DIM}(silence radio pendant le mouvement){RST}")

    t_start = time.time()
    last_disp = -1
    while True:
        elapsed = time.time() - t_start
        remain  = duration - elapsed
        if remain <= 0:
            break
        sec_int = int(remain) + 1
        if sec_int != last_disp:
            print(f"\r  {DIM}[{elapsed:5.1f}s | reste {sec_int:3d}s]{RST}  "
                  f"{CYN}mouvement en cours...{RST}        ",
                  end="", flush=True)
            last_disp = sec_int
        time.sleep(0.1)

    print(f"\r  {DIM}[{duration:5.1f}s | reste   0s]{RST}  "
          f"{GRN}fin du mouvement                          {RST}")

    stop_burst(ser, n=5)
    time.sleep(0.2)
    s = do_status_quick(ser)
    if s:
        ok(f"STM32 responsif : {fmt_status(s)}")
    else:
        warn("STM32 muet apres STOP. Possible : overrun firmware (ORE).")
        warn("Verifie que HAL_UART_ErrorCallback est bien dans main.c")

# ─────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────

_ser_global = None

def sigint_handler(signum, frame):
    print()
    warn("Ctrl+C --- STOP x5")
    if _ser_global is not None:
        try:
            stop_burst(_ser_global, n=5)
            _ser_global.close()
        except Exception:
            pass
    sys.exit(0)

def main():
    global _ser_global

    parser = argparse.ArgumentParser(description="Test moteurs mecanum (v3 - bypass PID/grille)")
    parser.add_argument("port", nargs="?", default="/dev/serial0")
    parser.add_argument("--baud",     type=int,   default=115200)
    parser.add_argument("--norm",     type=float, default=0.30,
                        help="Vitesse normalisee des roues (0..1, defaut 0.30)")
    parser.add_argument("--duration", type=float, default=6.0,
                        help="Duree de chaque test en secondes (defaut 6)")
    parser.add_argument("--pause",    type=float, default=0.8)
    parser.add_argument("--only-wheels", action="store_true",
                        help="Ne fait que les 8 tests de roue individuelle")
    parser.add_argument("--only-moves",  action="store_true",
                        help="Ne fait que les 6 tests de deplacement")
    args = parser.parse_args()

    all_tests = make_tests(args.norm)
    if args.only_wheels:
        tests = [t for t in all_tests if t['kind'] == 'wheel']
    elif args.only_moves:
        tests = [t for t in all_tests if t['kind'] == 'vel']
    else:
        tests = all_tests

    total = len(tests)
    est_time = total * (args.duration + args.pause + 1.5)

    print()
    sep()
    print(f"{BOLD}{CYN}  TEST MOTEURS MECANUM v3 --- DIAGNOSTIC INDIVIDUEL{RST}")
    print(f"{DIM}  Firmware requis : CMD_TEST_WHEEL + CMD_RAW_VELOCITY + ORE handler{RST}")
    sep()
    info(f"Port      : {args.port}  @{args.baud} baud")
    info(f"Vitesse   : norm = {args.norm} (sur les commandes wheel)")
    info(f"Duree     : {args.duration}s par test  x  {total} tests  ~  {est_time:.0f}s total")
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
    else:
        warn("Pas de reponse - STM32 allume ? Cable OK ? Firmware a jour ?")

    sep()
    info("Demarrage dans 3 secondes...")
    time.sleep(3.0)

    t_run_start = time.time()
    for idx, t in enumerate(tests):
        sep()
        kind_color = YLW if t['kind'] == 'wheel' else GRN
        print(f"\n  {BOLD}{kind_color}|-- [{idx+1}/{total}]  TEST {t['id']} --- {t['label']}{RST}")
        print(f"  {BOLD}{kind_color}+--{RST}")
        run_test(ser, t, args.duration)
        time.sleep(args.pause)

    sep()
    ok(f"Sequence complete --- {total} tests en {time.time()-t_run_start:.1f}s")
    stop_burst(ser, n=3)
    ser.close()
    ok("Port ferme. Fin.")
    print()


if __name__ == "__main__":
    main()
