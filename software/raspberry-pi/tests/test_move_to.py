#!/usr/bin/env python3
# -*- coding: ascii -*-

"""
test_move_to.py - Etape 3 : envoie une coordonnee absolue (x, y, theta)
                           et suit la convergence du PID position.

Sequence :
  1. CMD_RESET_ODOM  -> pose origine (0, 0, 0)
  2. CMD_MOVE_TO(x, y, theta)
  3. Boucle de polling CMD_GET_STATUS @ 10 Hz :
       - lit pose courante, erreur, etat machine, encodeurs
       - affiche en live
  4. Sortie de boucle quand state == DONE (cible atteinte + stabilisation)
     ou ESTOP / ERROR / timeout / Ctrl+C
  5. STOP de fin, recap : distance parcourue, erreur residuelle, temps

ATTENTION : asservissement actif et calibration TICKS_PER_METER probablement
sous-estimee. Le robot va parcourir physiquement environ 3 a 6 fois la
distance que l'odom mesure. Commencer par des cibles TRES PETITES (5 cm
maxi) pour ne pas casser quelque chose.

States renvoyes par le firmware (StatusPayload_t.state) :
  0 = IDLE
  1 = MOVING       (PID en train de pousser vers la cible)
  2 = SETTLING     (dans la tolerance, en attente confirmation)
  3 = DONE         (mouvement termine, RESP_DONE deja envoye)
  4 = ESTOP        (ARU)
  5 = ERROR        (hors grille p.ex.)

Usage :
  python3 test_move_to.py /dev/serial0 0.05 0.00
  python3 test_move_to.py /dev/serial0 0.10 0.05 --theta 0.0
  python3 test_move_to.py /dev/serial0 0.10 0   --timeout 20 --period 0.1
  python3 test_move_to.py /dev/serial0 0.05 0   --no-reset    (garde la pose courante)
"""

import argparse
import math
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

CMD_MOVE_TO      = 0x01
CMD_STOP         = 0x02
CMD_RESET_ODOM   = 0x03
CMD_GET_STATUS   = 0x04
CMD_SET_VELOCITY = 0x07

RESP_ACK         = 0xA1
RESP_DONE        = 0xA2
RESP_STATUS      = 0xA3
RESP_ERROR       = 0xA4

# Payload STATUS (53 octets) :
#   <fff   : vx_cmd, vy_cmd, wz_cmd
#   <iiii  : e1, e2, e3, e4 (encodeurs bruts)
#   <fff   : x, y, theta
#   <fff   : vx_meas, vy_meas, wz_meas
#   <B     : state machine (0=IDLE,1=MOVING,2=SETTLING,3=DONE,4=ESTOP,5=ERROR)
STATUS_FMT  = "<fffiiiiffffffB"
STATUS_SIZE = struct.calcsize(STATUS_FMT)   # 53

STATE_NAMES = {
    0: "IDLE",
    1: "MOVING",
    2: "SETTLING",
    3: "DONE",
    4: "ESTOP",
    5: "ERROR",
}

ERR_NAMES = {
    0x01: "ERR_CRC",
    0x02: "ERR_UNKNOWN_CMD",
    0x03: "ERR_BUSY",
    0x04: "ERR_OUT_BOUNDS",
    0x05: "ERR_ARU",
    0x06: "ERR_AX12",
}

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
def sep():   print("%s%s%s" % (DIM, "-" * 94, RST), flush=True)


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


# ============================================================================
# COMMANDES
# ============================================================================

def cmd_stop(ser):
    send(ser, CMD_STOP)
    r = recv(ser, 0.5)
    return r is not None and r[0] == RESP_ACK


def cmd_reset_odom(ser):
    send(ser, CMD_RESET_ODOM)
    r = recv(ser, 0.5)
    return r is not None and r[0] == RESP_ACK


def cmd_move_to(ser, x, y, theta):
    payload = struct.pack("<fff", x, y, theta)
    send(ser, CMD_MOVE_TO, payload)
    r = recv(ser, 0.6)
    if r is None:
        return False, "no_response"
    resp, data = r
    if resp == RESP_ACK:
        return True, "ack"
    if resp == RESP_ERROR and len(data) >= 1:
        return False, ERR_NAMES.get(data[0], "ERR_0x%02X" % data[0])
    return False, "unexpected_resp_0x%02X" % resp


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
     vx_meas, vy_meas, wz_meas,
     state) = struct.unpack_from(STATUS_FMT, data, 0)

    return dict(
        vx_cmd=vx_cmd, vy_cmd=vy_cmd, wz_cmd=wz_cmd,
        e1=e1, e2=e2, e3=e3, e4=e4,
        x=x, y=y, theta=theta,
        vx_meas=vx_meas, vy_meas=vy_meas, wz_meas=wz_meas,
        state=state, state_name=STATE_NAMES.get(state, "?"),
    )


def stop_burst(ser, n=5):
    """STOP repete pour fiabilite."""
    acked = False
    for i in range(n):
        ser.reset_input_buffer()
        if cmd_stop(ser):
            acked = True
            break
        time.sleep(0.08)
    return acked


# ============================================================================
# AFFICHAGE LIVE
# ============================================================================

def signed_delta_16(curr, prev):
    """Delta signe entre deux compteurs uint16 (gere l'overflow)."""
    d = (int(curr) - int(prev)) & 0xFFFF
    if d >= 32768:
        d -= 65536
    return d


def print_header():
    print("    %s%5s  %7s %7s   %7s %7s   %7s %7s   %s%7s %7s %7s %7s%s   %8s%s" % (
        BOLD,
        "t[s]",
        "x [m]", "y [m]",
        "ex [m]", "ey [m]",
        "vx_m", "vy_m",
        DIM,
        "vFL", "vFR", "vRL", "vRR",
        RST + BOLD,
        "state",
        RST,
    ), flush=True)
    print("    %s%5s  %7s %7s   %7s %7s   %7s %7s   %s%7s %7s %7s %7s%s   %8s%s" % (
        DIM,
        "",
        "world", "world",
        "world", "world",
        "m/s", "m/s",
        DIM,
        "m/s", "m/s", "m/s", "m/s",
        RST + DIM,
        "FSM",
        RST,
    ), flush=True)


def state_color(name):
    if name == "MOVING":   return CYN
    if name == "SETTLING": return YLW
    if name == "DONE":     return GRN
    if name == "IDLE":     return DIM
    if name in ("ESTOP", "ERROR"): return RED
    return RST


def print_row(t_rel, st, tx, ty, wheel_v):
    """wheel_v = tuple (vFL, vFR, vRL, vRR) en m/s, ou None si premier echantillon."""
    ex = tx - st["x"]
    ey = ty - st["y"]
    sc = state_color(st["state_name"])
    if wheel_v is None:
        wstr = "   --      --      --      --   "
    else:
        wFL, wFR, wRL, wRR = wheel_v
        wstr = "%+7.3f %+7.3f %+7.3f %+7.3f" % (wFL, wFR, wRL, wRR)
    print("    %5.2f  %s%+7.4f %+7.4f%s   %s%+7.4f %+7.4f%s   %s%+7.3f %+7.3f%s   %s%s%s   %s%-8s%s" % (
        t_rel,
        BOLD, st["x"], st["y"], RST,
        MAG,  ex, ey, RST,
        DIM,  st["vx_meas"], st["vy_meas"], RST,
        DIM,  wstr, RST,
        sc,   st["state_name"], RST,
    ), flush=True)


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


def main():
    global _ser_global

    parser = argparse.ArgumentParser(
        description="Test asservissement position : MOVE_TO(x, y, theta) + suivi convergence")
    parser.add_argument("port",      nargs="?", default="/dev/serial0")
    parser.add_argument("x",         type=float, help="Cible x en metres (repere monde)")
    parser.add_argument("y",         type=float, help="Cible y en metres (repere monde)")
    parser.add_argument("--theta",   type=float, default=0.0,
                        help="Cible theta en radians (ignoree par le firmware, 0 par defaut)")
    parser.add_argument("--baud",    type=int,   default=115200)
    parser.add_argument("--timeout", type=float, default=15.0,
                        help="Timeout max d'attente convergence (defaut 15 s)")
    parser.add_argument("--period",  type=float, default=0.10,
                        help="Periode de polling STATUS (defaut 0.10 s)")
    parser.add_argument("--no-reset", action="store_true",
                        help="Ne PAS faire CMD_RESET_ODOM avant le MOVE_TO")
    parser.add_argument("--ticks-per-meter", type=float, default=5126.0,
                        help="Calibration odom (TICKS_PER_METER firmware). "
                             "Doit correspondre a la valeur firmware. "
                             "Defaut 5126 (= 48*51 / (2*pi*0.076), calib=1.0).")
    args = parser.parse_args()

    print()
    sep()
    print("  %sTEST MOVE_TO -- asservissement position (x, y)%s" % (BOLD + CYN, RST))
    sep()
    info("Port      : %s @ %d baud" % (args.port, args.baud))
    info("Cible     : x = %+.4f m   y = %+.4f m   theta = %+.4f rad" %
         (args.x, args.y, args.theta))
    info("Timeout   : %.1f s    Polling : %.2f s" % (args.timeout, args.period))
    info("Reset odom: %s" % ("non" if args.no_reset else "oui (pose origine -> 0,0,0)"))
    sep()
    print("  %sASSERVISSEMENT 2D (x, y) - pas de rotation theta%s" % (BOLD + GRN, RST))
    print("    - Vitesse plafonnee a VELOCITY_CAP = 0.10 m/s")
    print("    - Zone morte sortie controleur : VELOCITY_DEADBAND = 0.015 m/s")
    print("    - Tolerance convergence : POS_TOLERANCE_M = 0.020 m")
    print("    - Le robot ne tourne PAS sur lui-meme (translation pure)")
    print("    - Si le robot derive trop : verifier equilibre roues dans le recap final")
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
        err("Pas de reponse - firmware a jour ? CMD_MOVE_TO compile ? STATUS 53 octets ?")
        ser.close()
        sys.exit(2)
    ok("STM32 repond : x=%+.4f y=%+.4f th=%+.4f  state=%s" %
       (st["x"], st["y"], st["theta"], st["state_name"]))

    # STOP de securite
    info("Envoi CMD_STOP initial")
    stop_burst(ser, n=3)

    # Reset odom optionnel
    if not args.no_reset:
        info("Envoi CMD_RESET_ODOM (pose origine -> 0,0,0)")
        if cmd_reset_odom(ser):
            ok("Odometrie remise a zero")
        else:
            warn("Pas d'ACK sur RESET_ODOM, on continue quand meme")

    # Demarrage
    info("Demarrage MOVE_TO dans 3 secondes... (Ctrl+C pour annuler)")
    time.sleep(3.0)

    # Envoi MOVE_TO
    success, reason = cmd_move_to(ser, args.x, args.y, args.theta)
    if not success:
        err("MOVE_TO refuse : %s" % reason)
        stop_burst(ser, n=3)
        ser.close()
        sys.exit(3)
    ok("MOVE_TO ACK -- robot en MOVING")

    # Boucle de suivi
    sep()
    print_header()

    t0 = time.time()
    last = t0
    final_state = None
    done_reason = None

    # Suivi par-roue : on garde les valeurs precedentes pour calculer
    # une vitesse instantanee par roue a chaque echantillon.
    prev_e   = None
    prev_t   = None
    # Stats : magnitudes (valeurs absolues) cumulees pour calculer la
    # moyenne par-roue sur la duree du mouvement -> verifier l'equilibre.
    wheel_v_abs_sum = [0.0, 0.0, 0.0, 0.0]
    wheel_v_count   = 0

    while True:
        elapsed = time.time() - t0
        if elapsed >= args.timeout:
            done_reason = "TIMEOUT"
            break

        now = time.time()
        wait = args.period - (now - last)
        if wait > 0:
            time.sleep(wait)
        last = time.time()

        st = cmd_get_status(ser)
        if st is None:
            warn("Pas de reponse au GET_STATUS")
            continue

        # Calcul vitesse par roue (m/s) a partir des deltas encodeurs
        t_now = time.time()
        wheel_v = None
        if prev_e is not None and prev_t is not None:
            dt = t_now - prev_t
            if dt > 1e-3:
                de = [
                    signed_delta_16(st["e1"], prev_e[0]),
                    signed_delta_16(st["e2"], prev_e[1]),
                    signed_delta_16(st["e3"], prev_e[2]),
                    signed_delta_16(st["e4"], prev_e[3]),
                ]
                wheel_v = tuple(d / dt / args.ticks_per_meter for d in de)
                # Cumul magnitudes pour stats de fin
                for i, v in enumerate(wheel_v):
                    wheel_v_abs_sum[i] += abs(v)
                wheel_v_count += 1

        prev_e = (st["e1"], st["e2"], st["e3"], st["e4"])
        prev_t = t_now

        print_row(elapsed, st, args.x, args.y, wheel_v)

        if st["state_name"] == "DONE":
            final_state = st
            done_reason = "DONE"
            break
        if st["state_name"] in ("ESTOP", "ERROR"):
            final_state = st
            done_reason = st["state_name"]
            break

    # STOP defensif
    sep()
    info("Fin de boucle (%s) -- STOP" % done_reason)
    stop_burst(ser, n=3)

    # Recap
    time.sleep(0.3)
    st_final = cmd_get_status(ser)
    if st_final is None:
        warn("Pas de pose finale lue")
        st_final = final_state

    sep()
    print("  %sRECAPITULATIF%s" % (BOLD + CYN, RST))
    sep()
    elapsed = time.time() - t0
    print("  Duree           : %.2f s" % elapsed)
    print("  Raison sortie   : %s" % done_reason)
    if st_final is not None:
        ex = args.x - st_final["x"]
        ey = args.y - st_final["y"]
        dist = math.sqrt(ex * ex + ey * ey)
        within = dist < 0.020   # POS_TOLERANCE_M firmware
        col = GRN if within else YLW
        print("  Cible           : (%+.4f, %+.4f)" % (args.x, args.y))
        print("  Pose finale     : (%s%+.4f%s, %s%+.4f%s)   theta = %+.4f rad" % (
            BOLD, st_final["x"], RST, BOLD, st_final["y"], RST, st_final["theta"]))
        print("  Erreur residu.  : (%+.4f, %+.4f)   dist = %s%.4f m%s   (tol = 0.020 m)" % (
            ex, ey, col, dist, RST))
        print("  State final     : %s%s%s" % (state_color(st_final["state_name"]),
                                              st_final["state_name"], RST))

    # --- Stats vitesse par roue ---
    if wheel_v_count > 0:
        avg = [s / wheel_v_count for s in wheel_v_abs_sum]
        # Reference : moyenne des 4 roues
        ref = sum(avg) / 4.0
        # Ecarts relatifs vs reference
        deviations = [(v - ref) / ref * 100.0 if ref > 1e-6 else 0.0 for v in avg]
        max_dev = max(abs(d) for d in deviations)
        balance_col = GRN if max_dev < 15.0 else (YLW if max_dev < 35.0 else RED)
        print()
        print("  %sEQUILIBRE ROUES (moyenne |vitesse| sur %d echantillons) :%s" % (
            BOLD, wheel_v_count, RST))
        labels = ["FL", "FR", "RL", "RR"]
        for i, lbl in enumerate(labels):
            print("    %s  : |v_avg| = %s%.4f m/s%s   ecart vs moyenne = %s%+.1f%%%s" % (
                lbl,
                BOLD, avg[i], RST,
                balance_col if abs(deviations[i]) < 15 else YLW,
                deviations[i], RST,
            ))
        verdict = "BON" if max_dev < 15.0 else ("MOYEN" if max_dev < 35.0 else "MAUVAIS")
        print("    %sVerdict equilibre roues : %s%s%s  (ecart max %.1f%%)" % (
            DIM, balance_col, verdict, RST, max_dev))
    sep()

    ser.close()
    ok("Port serie ferme. Fin.")
    print()


if __name__ == "__main__":
    main()
