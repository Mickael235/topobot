#!/usr/bin/env python3
# -*- coding: ascii -*-

"""
test_encoders.py - Etape 1 : valider les encodeurs via l'odometrie

Principe : on ne commande PAS les moteurs. On pousse le robot a la main
et on lit en boucle la pose calculee par l'odometrie embarquee. On valide :

  1. Sens de rotation de chaque encodeur
       - pousse roue FL vers l'avant : le compteur e1 doit augmenter (ou
         diminuer de maniere coherente). Idem pour FR, RL, RR.
  2. Echelle (TICKS_PER_METER)
       - pousse le robot tout droit sur exactement 1 m mesure au metre :
         x final doit etre proche de +1.00 m, y proche de 0.
  3. Signe de theta
       - fais tourner le robot sur lui-meme dans le sens trigo (vu de
         dessus, sens anti-horaire) : theta doit AUGMENTER.

Aucune commande SET_VELOCITY n'est envoyee. Les moteurs restent libres
(Mecanum_Stop a ete envoye en init), tu peux donc rouler le robot a la
main sans forcer contre le couple moteur.

Usage :
  python3 test_encoders.py
  python3 test_encoders.py /dev/serial0
  python3 test_encoders.py /dev/serial0 --period 0.1
  python3 test_encoders.py /dev/serial0 --reset      (reset pose au demarrage)
"""

import argparse
import serial
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

RESP_ACK         = 0xA1
RESP_STATUS      = 0xA3
RESP_ERROR       = 0xA4

# Layout du payload STATUS (firmware etendu - 52 octets)
#   <fff : vx_cmd, vy_cmd, wz_cmd       (consigne courante)
#   <iiii: e1, e2, e3, e4               (compteurs encodeurs bruts, FL FR RL RR)
#   <fff : x, y, theta                  (pose odometrie repere monde)
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
def sep():   print("%s%s%s" % (DIM, "-" * 78, RST), flush=True)


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


def cmd_get_status(ser):
    send(ser, CMD_GET_STATUS)
    r = recv(ser, 0.5)
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


# ============================================================================
# DELTA SIGNE 16 BITS
# ============================================================================

def signed_delta_16(curr, prev):
    """Le STM32 renvoie le compteur en uint16. Pour suivre proprement les
    rotations dans les deux sens, on calcule un delta signe en se basant sur
    la regle classique (cnt - prev) modulo 65536, ramene dans [-32768, +32767].
    Ce delta represente le nombre de ticks accumules entre deux lectures."""
    d = (curr - prev) & 0xFFFF
    if d >= 32768:
        d -= 65536
    return d


# ============================================================================
# AFFICHAGE
# ============================================================================

HEADER = (
    "  %-6s %s%10s %10s %10s %10s%s   %s%8s %8s %9s%s   %s%9s %9s %9s%s\n"
    "  %-6s %s%10s %10s %10s %10s%s   %s%8s %8s %9s%s   %s%9s %9s %9s%s"
)

def print_header():
    print(HEADER % (
        "",        BOLD, "e1 FL",  "e2 FR",  "e3 RL",  "e4 RR",  RST,
                   BOLD, "x [m]",  "y [m]",  "th [rad]",       RST,
                   BOLD, "vx_meas","vy_meas","wz_meas",         RST,
        "",        DIM,  "(raw)",  "(raw)",  "(raw)",  "(raw)",  RST,
                   DIM,  "world",  "world",  "world",          RST,
                   DIM,  "m/s",    "m/s",    "rad/s",          RST,
    ), flush=True)
    sep()


def fmt_row(t0, st, prev_enc, totals):
    elapsed = time.time() - t0

    # deltas signes par rapport au tick precedent (juste pour info)
    de = [signed_delta_16(st["e%d" % i], prev_enc[i - 1]) for i in (1, 2, 3, 4)]
    for i in range(4):
        totals[i] += de[i]

    # mise en couleur de theta selon signe / amplitude
    th_col = GRN if st["theta"] >= 0 else YLW

    line = (
        "  t=%5.1fs  "
        "%se1=%+7d%s  %se2=%+7d%s  %se3=%+7d%s  %se4=%+7d%s   "
        "%sx=%+7.3f  y=%+7.3f  th=%s%+7.3f%s%s   "
        "%svm=(%+6.3f,%+6.3f,%+6.3f)%s"
    ) % (
        elapsed,
        CYN, st["e1"], RST,
        CYN, st["e2"], RST,
        CYN, st["e3"], RST,
        CYN, st["e4"], RST,
        BOLD, st["x"], st["y"], th_col, st["theta"], RST, RST,
        MAG, st["vx_meas"], st["vy_meas"], st["wz_meas"], RST,
    )
    return line, de


# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="Test des encodeurs via odometrie")
    parser.add_argument("port",   nargs="?", default="/dev/serial0")
    parser.add_argument("--baud", type=int,   default=115200)
    parser.add_argument("--period", type=float, default=0.1,
                        help="Periode de polling en secondes (defaut 0.1 s)")
    parser.add_argument("--reset", action="store_true",
                        help="Envoyer CMD_RESET_ODOM au demarrage")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Duree max en secondes (0 = jusqu'a Ctrl+C)")
    args = parser.parse_args()

    print()
    sep()
    print("  %s%s%s" % (BOLD + CYN, "TEST ENCODEURS / ODOMETRIE", RST))
    sep()
    info("Port    : %s @ %d baud" % (args.port, args.baud))
    info("Periode : %.2f s entre deux polls" % args.period)
    if args.duration > 0:
        info("Duree   : %.1f s max" % args.duration)
    else:
        info("Duree   : illimitee (Ctrl+C pour quitter)")
    sep()
    print("  %sProtocole :%s" % (BOLD, RST))
    print("    1. Poser le robot, NE PAS allumer la puissance moteurs (PA5)")
    print("       sinon Mecanum_SetVelocity(0,0) cale les Sabertooth, c'est OK")
    print("       mais tu pousseras contre les moteurs freines.")
    print("    2. Pousser doucement le robot dans une direction (avant, droite,")
    print("       rotation) en regardant les compteurs et la pose.")
    print("    3. Verifier :")
    print("       %s- chaque encodeur change quand sa roue tourne%s"   % (GRN, RST))
    print("       %s- x augmente quand on pousse vers l'avant%s"        % (GRN, RST))
    print("       %s- y change quand on pousse lateralement%s"          % (GRN, RST))
    print("       %s- theta augmente en rotation trigonometrique (CCW)%s" % (GRN, RST))
    sep()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        err("Impossible d'ouvrir %s : %s" % (args.port, e))
        sys.exit(1)

    time.sleep(0.3)
    ser.reset_input_buffer()

    # Ping
    info("Ping STM32 (GET_STATUS)")
    st = cmd_get_status(ser)
    if st is None:
        err("Pas de reponse - firmware a jour ? Cable OK ?")
        ser.close()
        sys.exit(2)
    ok("STM32 repond : x=%+.3f y=%+.3f th=%+.3f  e=(%d,%d,%d,%d)" %
       (st["x"], st["y"], st["theta"], st["e1"], st["e2"], st["e3"], st["e4"]))

    # Securite : envoyer STOP (consigne zero) avant de commencer
    info("Envoi CMD_STOP (consigne = 0)")
    cmd_stop(ser)

    if args.reset:
        info("Envoi CMD_RESET_ODOM (pose = 0, encodeurs = 0)")
        if cmd_reset_odom(ser):
            ok("Odometrie remise a zero")
        else:
            warn("Pas d'ACK sur RESET_ODOM (commande supportee ?)")

    sep()
    print_header()

    st = cmd_get_status(ser)
    if st is None:
        err("Statut indisponible apres init")
        ser.close()
        sys.exit(3)

    prev_enc = [st["e1"], st["e2"], st["e3"], st["e4"]]
    totals   = [0, 0, 0, 0]

    t0 = time.time()
    last = t0

    try:
        while True:
            if args.duration > 0 and (time.time() - t0) >= args.duration:
                info("Duree atteinte, fin.")
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

            line, de = fmt_row(t0, st, prev_enc, totals)
            print(line, flush=True)

            prev_enc = [st["e1"], st["e2"], st["e3"], st["e4"]]

    except KeyboardInterrupt:
        print()
        info("Ctrl+C - bilan final :")

    sep()
    st = cmd_get_status(ser)
    if st is not None:
        print("  %sPose finale :%s   x = %s%+.3f m%s   y = %s%+.3f m%s   theta = %s%+.3f rad%s (%.1f deg)" % (
            BOLD, RST,
            GRN, st["x"], RST,
            GRN, st["y"], RST,
            GRN, st["theta"], RST,
            st["theta"] * 180.0 / 3.14159265,
        ))
        print("  %sCompteurs bruts (FL,FR,RL,RR) :%s e=(%d, %d, %d, %d)" % (
            BOLD, RST, st["e1"], st["e2"], st["e3"], st["e4"]))
        print("  %sTotaux deltas signes accumules:%s d=(%+d, %+d, %+d, %+d)" % (
            BOLD, RST, totals[0], totals[1], totals[2], totals[3]))
    sep()

    ser.close()
    ok("Port serie ferme. Fin.")
    print()


if __name__ == "__main__":
    main()
