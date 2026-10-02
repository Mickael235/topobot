#!/usr/bin/env python3
# -*- coding: ascii -*-

"""
test_odom_velocity.py - Etape 2 : valider la lecture odometrique en variant
                                   les vitesses commandees.

Principe : pour chaque mouvement (AVANT / ARRIERE / GAUCHE / DROITE) et pour
chaque vitesse de la liste --speeds, on :

  1. Envoie CMD_RESET_ODOM           (pose = 0, encodeurs = 0)
  2. Envoie CMD_SET_VELOCITY(vx,vy,0) (consigne maintenue par la main loop 100 Hz)
  3. Echantillonne CMD_GET_STATUS pendant --duration secondes
     -> on lit a chaque tour vx_cmd, vy_cmd, encodeurs, pose (x,y,theta)
        et vitesses mesurees par odometrie (vx_meas, vy_meas, wz_meas)
  4. Envoie CMD_STOP                 (consigne = 0)
  5. Attend --pause seconde puis lit la pose finale
  6. Calcule pour ce segment :
        - moyenne de la vitesse mesuree en regime etabli (apres 0.5 s d'accel)
        - ratio meas/cmd (devrait etre proche de 1.0)
        - distance parcourue (x ou y selon l'axe)
        - distance theorique = vx_cmd * duration
        - erreur relative

A la fin, un tableau recapitulatif permet de valider que l'odometrie suit
correctement la consigne sur la plage de vitesses testee.

ATTENTION : le robot bouge reellement pendant le test. Predire l'espace
necessaire : vmax * duration * 2 (aller + retour) ~ 1 m pour vx max 0.20 m/s
sur 2.5 s. Prevoir 1.5 m libre devant et 1 m sur les cotes.

Convention de l'utilisateur (a verifier physiquement pendant le test) :
   avant   : vx > 0
   arriere : vx < 0
   gauche  : vy < 0
   droite  : vy > 0

Usage :
  python3 test_odom_velocity.py
  python3 test_odom_velocity.py /dev/serial0
  python3 test_odom_velocity.py /dev/serial0 --speeds 0.10,0.15,0.20 --duration 2.5
  python3 test_odom_velocity.py /dev/serial0 --only avant
  python3 test_odom_velocity.py /dev/serial0 --only lateral --speeds 0.10,0.15
"""

import argparse
import serial
import signal
import struct
import sys
import time
from datetime import datetime

# ============================================================================
# PROTOCOLE
# ============================================================================

PROTO_START      = 0xAA

CMD_STOP         = 0x02
CMD_RESET_ODOM   = 0x03
CMD_GET_STATUS   = 0x04
CMD_SET_VELOCITY = 0x07

RESP_ACK         = 0xA1
RESP_STATUS      = 0xA3
RESP_ERROR       = 0xA4

# Payload STATUS etendu (52 octets)
#   <fff : vx_cmd, vy_cmd, wz_cmd
#   <iiii: e1, e2, e3, e4               (encodeurs bruts FL FR RL RR)
#   <fff : x, y, theta                  (pose repere monde)
#   <fff : vx_meas, vy_meas, wz_meas    (vitesses mesurees repere robot)
STATUS_FMT  = "<fffiiiiffffff"
STATUS_SIZE = struct.calcsize(STATUS_FMT)   # 52

# ============================================================================
# COULEURS
# ============================================================================

RST  = "\033[0m"
BOLD = "\033[1m"
DIM  = "\033[2m"

GRN = "\033[92m"
RED = "\033[91m"
YLW = "\033[93m"
CYN = "\033[96m"
BLU = "\033[94m"
MAG = "\033[95m"


def ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def log(symbol, color, msg):
    print("%s[%s]%s %s%s  %s%s" %
          (DIM, ts(), RST, color, symbol, msg, RST), flush=True)


def ok(m):   log("OK",   GRN, m)
def err(m):  log("ERR",  RED, m)
def warn(m): log("WARN", YLW, m)
def info(m): log("INFO", CYN, m)
def tx(m):   log("TX",   BLU, m)
def sep():   print("%s%s%s" % (DIM, "-" * 92, RST), flush=True)


# ============================================================================
# TRAMES
# ============================================================================

def crc(cmd, payload):
    c = cmd ^ len(payload)
    for b in payload:
        c ^= b
    return c & 0xFF


def frame(cmd, payload=b""):
    return bytes([PROTO_START, cmd, len(payload)]) + payload + bytes([crc(cmd, payload)])


def send(ser, cmd, payload=b""):
    f = frame(cmd, payload)
    ser.write(f)
    ser.flush()
    return f


def recv(ser, timeout=0.5):
    ser.timeout = 0.05
    deadline = time.time() + timeout

    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] != PROTO_START:
            continue

        hdr = ser.read(2)
        if len(hdr) != 2:
            continue

        resp = hdr[0]
        ln   = hdr[1]
        if ln > 64:
            continue

        rest = ser.read(ln + 1)
        if len(rest) != ln + 1:
            continue

        data    = rest[:ln]
        crc_rx  = rest[ln]
        if crc_rx != crc(resp, data):
            continue

        return resp, data

    return None


def cmd_stop(ser):
    send(ser, CMD_STOP)
    r = recv(ser, 0.5)
    return r is not None and r[0] == RESP_ACK


def cmd_reset_odom(ser):
    send(ser, CMD_RESET_ODOM)
    r = recv(ser, 0.5)
    return r is not None and r[0] == RESP_ACK


def cmd_set_velocity(ser, vx, vy, wz):
    payload = struct.pack("<fff", vx, vy, wz)
    send(ser, CMD_SET_VELOCITY, payload)
    r = recv(ser, 0.5)
    return r is not None and r[0] == RESP_ACK


def cmd_get_status(ser):
    send(ser, CMD_GET_STATUS)
    r = recv(ser, 0.4)
    if r is None:
        return None
    resp, data = r
    if resp != RESP_STATUS or len(data) < STATUS_SIZE:
        return None

    (vx_cmd, vy_cmd, wz_cmd,
     e1, e2, e3, e4,
     x, y, theta,
     vx_meas, vy_meas, wz_meas) = struct.unpack_from(STATUS_FMT, data, 0)

    return dict(
        vx_cmd=vx_cmd, vy_cmd=vy_cmd, wz_cmd=wz_cmd,
        e1=e1, e2=e2, e3=e3, e4=e4,
        x=x, y=y, theta=theta,
        vx_meas=vx_meas, vy_meas=vy_meas, wz_meas=wz_meas,
    )


def stop_burst(ser, n=5):
    """STOP repete pour fiabilite (entre chaque tentative on vide aussi l'entree)."""
    acked = False
    for i in range(n):
        ser.reset_input_buffer()
        if cmd_stop(ser):
            acked = True
            break
        time.sleep(0.08)
    return acked


# ============================================================================
# CATALOGUE DES MOUVEMENTS
# ============================================================================

# (key, label, vx_factor, vy_factor, axis_for_stats)
#   axis_for_stats = "x" ou "y" : sur quel axe on mesure la distance parcourue
MOVES = {
    "avant":   ("AVANT     (vx > 0)", +1,  0, "x"),
    "arriere": ("ARRIERE   (vx < 0)", -1,  0, "x"),
    "gauche":  ("GAUCHE    (vy < 0)",  0, -1, "y"),
    "droite":  ("DROITE    (vy > 0)",  0, +1, "y"),
}

DEFAULT_SEQUENCE = ["avant", "arriere", "gauche", "droite"]


# ============================================================================
# AFFICHAGE LIVE
# ============================================================================

def print_live_header():
    print("    %s%6s  %8s %8s  %8s %8s  %8s %8s  %8s%s" % (
        BOLD,
        "t[s]",
        "vx_cmd", "vx_meas",
        "vy_cmd", "vy_meas",
        "x [m]",  "y [m]",
        "wz_meas",
        RST,
    ), flush=True)


def print_live_row(t_rel, st):
    print("    %6.2f  %s%+8.3f %+8.3f%s  %s%+8.3f %+8.3f%s  %s%+8.3f %+8.3f%s  %s%+8.3f%s" % (
        t_rel,
        CYN, st["vx_cmd"], st["vx_meas"], RST,
        MAG, st["vy_cmd"], st["vy_meas"], RST,
        BOLD, st["x"],   st["y"],   RST,
        DIM, st["wz_meas"], RST,
    ), flush=True)


# ============================================================================
# UN SEGMENT
# ============================================================================

def run_segment(ser, label, vx_cmd, vy_cmd, duration, period, pause):
    """Joue un segment vitesse, echantillonne STATUS, retourne stats.

    Retourne un dict avec : label, vx_cmd, vy_cmd, axis, n_samples, mean_meas,
    final_x, final_y, expected_distance, ratio, err_rel, samples (liste brute).
    """
    sep()
    print("  %s%s%s   consigne : vx=%+.3f m/s   vy=%+.3f m/s   pendant %.1f s" %
          (BOLD + MAG, label, RST, vx_cmd, vy_cmd, duration))
    print()

    # 1. Reset odometrie pour mesurer la distance proprement
    if not cmd_reset_odom(ser):
        warn("RESET_ODOM sans ACK -- on continue quand meme")

    # 2. Envoi consigne
    if not cmd_set_velocity(ser, vx_cmd, vy_cmd, 0.0):
        err("SET_VELOCITY refuse, segment annule")
        return None

    # 3. Echantillonnage
    print_live_header()
    samples = []
    t0 = time.time()
    next_tick = t0

    while True:
        elapsed = time.time() - t0
        if elapsed >= duration:
            break

        # Aligner l'echantillonnage sur la grille pour eviter la derive
        next_tick += period
        st = cmd_get_status(ser)
        if st is not None:
            samples.append((elapsed, st))
            print_live_row(elapsed, st)

        # Attendre le prochain tick
        wait = next_tick - time.time()
        if wait > 0:
            time.sleep(wait)

    # 4. STOP
    stop_burst(ser, n=3)

    # 5. Pause de decantation + relecture pose finale
    time.sleep(pause)
    final = cmd_get_status(ser)

    # 6. Calcul stats : on saute les 0.5 premieres secondes (acceleration)
    SKIP_S = 0.5
    steady = [(t, s) for (t, s) in samples if t >= SKIP_S]

    is_x = abs(vx_cmd) >= abs(vy_cmd)
    axis = "x" if is_x else "y"
    cmd_val = vx_cmd if is_x else vy_cmd
    meas_key = "vx_meas" if is_x else "vy_meas"

    if steady:
        mean_meas = sum(s[meas_key] for (_, s) in steady) / len(steady)
    else:
        mean_meas = 0.0

    if final is None:
        warn("Pas de pose finale lue")
        final_x = final_y = 0.0
    else:
        final_x = final["x"]
        final_y = final["y"]

    measured_distance = final_x if is_x else final_y
    expected_distance = cmd_val * duration

    ratio   = (mean_meas / cmd_val) if abs(cmd_val) > 1e-9 else 0.0
    err_rel = ((measured_distance - expected_distance) / expected_distance
               if abs(expected_distance) > 1e-9 else 0.0)

    # 7. Resume du segment
    print()
    print("  %s>> regime etabli (n=%d echantillons apres %.1fs) :%s   "
          "mean(%s_meas) = %s%+.4f m/s%s    ratio = %s%+.3f%s" % (
              DIM, len(steady), SKIP_S, RST,
              axis,
              GRN if (0.85 <= abs(ratio) <= 1.15) else YLW, mean_meas, RST,
              GRN if (0.85 <= abs(ratio) <= 1.15) else YLW, ratio, RST,
          ))
    print("  %s>> distance %s :%s   mesuree = %s%+.4f m%s   attendue = %+.4f m   "
          "erreur = %s%+.1f %%%s" % (
              DIM, axis, RST,
              GRN if abs(err_rel) <= 0.20 else YLW, measured_distance, RST,
              expected_distance,
              GRN if abs(err_rel) <= 0.20 else YLW, err_rel * 100.0, RST,
          ))

    return dict(
        label=label,
        vx_cmd=vx_cmd, vy_cmd=vy_cmd,
        axis=axis, cmd_val=cmd_val,
        n_samples=len(steady),
        mean_meas=mean_meas,
        measured_distance=measured_distance,
        expected_distance=expected_distance,
        ratio=ratio,
        err_rel=err_rel,
        final_pose=(final_x, final_y),
    )


# ============================================================================
# RECAPITULATIF
# ============================================================================

def print_summary(results):
    sep()
    print("  %sRECAPITULATIF -- consigne vs odometrie%s" % (BOLD + CYN, RST))
    sep()
    print("  %s%-20s %6s %10s %10s %8s %10s %10s %8s%s" % (
        BOLD,
        "Mouvement", "axe",
        "cmd[m/s]", "meas[m/s]", "ratio",
        "d_meas[m]", "d_th[m]", "err [%]",
        RST,
    ))
    print("  %s%s%s" % (DIM, "-" * 92, RST))

    for r in results:
        if r is None:
            continue

        ratio_col = GRN if (0.85 <= abs(r["ratio"]) <= 1.15) else YLW
        err_col   = GRN if abs(r["err_rel"]) <= 0.20      else YLW

        print("  %-20s %6s %+10.3f %s%+10.3f%s %s%+8.3f%s %+10.4f %+10.4f %s%+8.1f%s" % (
            r["label"], r["axis"],
            r["cmd_val"],
            ratio_col, r["mean_meas"], RST,
            ratio_col, r["ratio"],     RST,
            r["measured_distance"], r["expected_distance"],
            err_col, r["err_rel"] * 100.0, RST,
        ))
    sep()


# ============================================================================
# MAIN
# ============================================================================

_ser_global = None


def sigint_handler(signum, frame):
    print()
    warn("Ctrl+C -- STOP en rafale")
    if _ser_global is not None:
        try:
            stop_burst(_ser_global, n=5)
            _ser_global.close()
        except Exception:
            pass
    sys.exit(130)


def parse_speeds(s):
    out = []
    for tok in s.split(","):
        tok = tok.strip()
        if not tok:
            continue
        try:
            v = float(tok)
        except ValueError:
            raise argparse.ArgumentTypeError("speed invalide : %r" % tok)
        if v <= 0.0 or v > 0.40:
            raise argparse.ArgumentTypeError(
                "speed hors plage [0, 0.40] m/s : %r" % tok)
        out.append(v)
    if not out:
        raise argparse.ArgumentTypeError("--speeds vide")
    return out


def main():
    global _ser_global

    parser = argparse.ArgumentParser(
        description="Validation odometrie en faisant varier les vitesses")
    parser.add_argument("port", nargs="?", default="/dev/serial0")
    parser.add_argument("--baud",     type=int,   default=115200)
    parser.add_argument("--speeds",   type=parse_speeds, default=[0.10, 0.15, 0.20],
                        help="Liste de vitesses a tester en m/s (defaut: 0.10,0.15,0.20)")
    parser.add_argument("--duration", type=float, default=2.5,
                        help="Duree de chaque segment en secondes (defaut 2.5)")
    parser.add_argument("--pause",    type=float, default=1.0,
                        help="Pause apres STOP avant lecture pose finale (defaut 1.0)")
    parser.add_argument("--period",   type=float, default=0.10,
                        help="Periode d'echantillonnage STATUS pendant le segment (defaut 0.10s)")
    parser.add_argument("--only",     type=str,   default="all",
                        help="Filtre : avant/arriere/gauche/droite/lateral/all (defaut all)")
    args = parser.parse_args()

    # Selection mouvements
    if args.only == "all":
        sequence = DEFAULT_SEQUENCE
    elif args.only == "lateral":
        sequence = ["gauche", "droite"]
    elif args.only in MOVES:
        sequence = [args.only]
    else:
        err("--only inconnu : %s" % args.only)
        err("Valeurs valides : avant, arriere, gauche, droite, lateral, all")
        sys.exit(1)

    total_segments = len(sequence) * len(args.speeds)
    est_time       = total_segments * (args.duration + args.pause + 0.6)

    print()
    sep()
    print("  %sTEST ODOMETRIE -- VITESSES VARIABLES%s" % (BOLD + CYN, RST))
    sep()
    info("Port       : %s @ %d baud" % (args.port, args.baud))
    info("Vitesses   : %s m/s" % ", ".join("%.3f" % v for v in args.speeds))
    info("Duree/seg  : %.2f s    Pause: %.2f s   Sample: %.2f s" %
         (args.duration, args.pause, args.period))
    info("Sequence   : %s" % " -> ".join(sequence))
    info("Segments   : %d  (~ %.0f s total)" % (total_segments, est_time))
    sep()
    print("  %sATTENTION : le robot va reellement bouger.%s" % (YLW + BOLD, RST))
    print("    - Verifier qu'il y a au moins 1.5 m libre devant et 1 m sur les cotes.")
    print("    - Alimentation puissance ON (PA5).")
    print("    - Toujours pret a couper l'ARU si comportement anormal.")
    sep()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        err("Impossible d'ouvrir %s : %s" % (args.port, e))
        sys.exit(1)

    _ser_global = ser
    signal.signal(signal.SIGINT, sigint_handler)

    time.sleep(0.3)
    ser.reset_input_buffer()

    # Ping
    info("Ping STM32 (GET_STATUS)")
    st = cmd_get_status(ser)
    if st is None:
        err("Pas de reponse -- firmware a jour ? Cable OK ? Power ON ?")
        ser.close()
        sys.exit(2)
    ok("STM32 repond : x=%+.3f y=%+.3f th=%+.3f  e=(%d,%d,%d,%d)" %
       (st["x"], st["y"], st["theta"], st["e1"], st["e2"], st["e3"], st["e4"]))

    # Securite : STOP avant de commencer
    info("Envoi CMD_STOP initial")
    stop_burst(ser, n=3)

    info("Demarrage dans 3 secondes... (Ctrl+C pour annuler)")
    time.sleep(3.0)

    # Sequence : mouvement par mouvement, vitesse par vitesse
    results = []
    t_run_start = time.time()

    for key in sequence:
        label, fx, fy, _axis = MOVES[key]
        for speed in args.speeds:
            vx = speed * fx
            vy = speed * fy
            r  = run_segment(ser, label, vx, vy,
                             args.duration, args.period, args.pause)
            results.append(r)

    # STOP final + relecture
    sep()
    info("Sequence terminee, STOP final")
    stop_burst(ser, n=3)
    st = cmd_get_status(ser)
    if st is not None:
        ok("Consigne finale : vx=%+.3f vy=%+.3f wz=%+.3f" %
           (st["vx_cmd"], st["vy_cmd"], st["wz_cmd"]))

    # Recap
    print()
    print_summary(results)
    print()

    elapsed_total = time.time() - t_run_start
    ok("Test odom termine en %.1f s." % elapsed_total)
    ser.close()
    print()


if __name__ == "__main__":
    main()
