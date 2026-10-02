#!/usr/bin/env python3
# -*- coding: ascii -*-

"""
test_ax12.py - Test et diagnostic du servo AX12 sur UART5 (PC12)

Sequence :
  1. Ping STM32 (GET_STATUS) + affichage etat firmware
  2. CMD_DEPLOY_PRISM  (0x10) -> AX12 position deployee (700)
  3. Attente + verification visuelle
  4. CMD_RETRACT_PRISM  (0x11) -> AX12 position rentree (200)
  5. Attente + verification visuelle
  6. Optionnel : boucle N cycles deploy/retract

Usage :
  python test_ax12.py /dev/serial0
  python test_ax12.py COM6 --cycles 3 --pause 2.0
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

PROTO_START       = 0xAA

CMD_STOP          = 0x02
CMD_GET_STATUS    = 0x04
CMD_DEPLOY_PRISM  = 0x10
CMD_RETRACT_PRISM = 0x11

RESP_ACK    = 0xA1
RESP_STATUS = 0xA3
RESP_ERROR  = 0xA4

ERR_NAMES = {
    0x01: "ERR_CRC",
    0x02: "ERR_UNKNOWN_CMD",
    0x03: "ERR_BUSY",
    0x04: "ERR_OUT_BOUNDS",
    0x05: "ERR_ARU",
    0x06: "ERR_AX12",
}

STATE_NAMES = {
    0: "IDLE",
    1: "MOVING",
    2: "SETTLING",
    3: "DONE",
    4: "ESTOP",
    5: "ERROR",
}

# ============================================================================
# COULEURS
# ============================================================================

RST  = "\033[0m"
BOLD = "\033[1m"
DIM  = "\033[2m"
GRN  = "\033[92m"
RED  = "\033[91m"
YLW  = "\033[93m"
CYN  = "\033[96m"
BLU  = "\033[94m"
MGT  = "\033[95m"


def ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def log(symbol, color, msg):
    print("%s[%s]%s %s%s  %s%s" %
          (DIM, ts(), RST, color, symbol, msg, RST), flush=True)

def ok(m):   log("OK",   GRN, m)
def err(m):  log("ERR",  RED, m)
def warn(m): log("WARN", YLW, m)
def info(m): log("INFO", CYN, m)
def dbg(m):  log("DBG",  DIM, m)
def sep():   print("%s%s%s" % (DIM, "-" * 60, RST), flush=True)

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
# PARSING STATUS
# ============================================================================

def parse_status(data):
    """Parse le payload STATUS (53 octets)."""
    if len(data) < 53:
        return None
    fields = struct.unpack('<3f 4i 3f 3f B', data[:53])
    return {
        'vx_cmd':  fields[0],  'vy_cmd':  fields[1],  'wz_cmd':  fields[2],
        'e1':      fields[3],  'e2':      fields[4],
        'e3':      fields[5],  'e4':      fields[6],
        'x':       fields[7],  'y':       fields[8],  'theta':   fields[9],
        'vx_meas': fields[10], 'vy_meas': fields[11], 'wz_meas': fields[12],
        'state':   fields[13],
    }

def show_status(st):
    """Affiche les infos STATUS du STM32."""
    state_name = STATE_NAMES.get(st['state'], "???(%d)" % st['state'])
    info("  Etat asservissement : %s%s%s" % (BOLD, state_name, RST))
    info("  Odom  : x=%.3f m  y=%.3f m  th=%.1f deg" %
         (st['x'], st['y'], st['theta'] * 57.2958))
    info("  Enc   : RL=%d  RR=%d  (FL=%d  FR=%d)" %
         (st['e3'], st['e4'], st['e1'], st['e2']))

# ============================================================================
# COMMANDES
# ============================================================================

def cmd_get_status(ser):
    send(ser, CMD_GET_STATUS)
    return recv(ser, 0.5)

def cmd_deploy(ser):
    t0 = time.time()
    send(ser, CMD_DEPLOY_PRISM)
    r = recv(ser, 1.0)
    dt = (time.time() - t0) * 1000
    return r, dt

def cmd_retract(ser):
    t0 = time.time()
    send(ser, CMD_RETRACT_PRISM)
    r = recv(ser, 1.0)
    dt = (time.time() - t0) * 1000
    return r, dt

# ============================================================================
# DIAGNOSTIC
# ============================================================================

def check_response(r, dt_ms, cmd_name):
    """Analyse la reponse et retourne (ack_ok, erreur_ax12)."""
    if r is None:
        err("%s : pas de reponse du STM32 (timeout)" % cmd_name)
        return False, False

    resp, data = r

    if resp == RESP_ACK:
        ok("%s : ACK en %.0f ms" % (cmd_name, dt_ms))
        warn("  NOTE: ACK = le STM32 a envoye la trame sur UART5.")
        warn("  Cela ne garantit PAS que le servo AX12 l'a recue !")
        return True, False

    if resp == RESP_ERROR and len(data) >= 1:
        ename = ERR_NAMES.get(data[0], "0x%02X" % data[0])
        err("%s : ERREUR %s (en %.0f ms)" % (cmd_name, ename, dt_ms))
        if data[0] == 0x06:
            err("  -> ERR_AX12 : HAL_UART_Transmit a echoue")
            err("     Probleme materiel sur UART5 (PC12)")
            return False, True
        return False, False

    warn("%s : reponse inattendue 0x%02X" % (cmd_name, resp))
    return False, False

def print_checklist():
    """Affiche la checklist de debug complete."""
    print()
    print("  %sCHECKLIST DEBUG AX12%s" % (BOLD + RED, RST))
    sep()
    print("  %s1. CABLAGE%s" % (BOLD, RST))
    print("     - PC12 (UART5_TX) connecte au fil DATA de l'AX12 ?")
    print("     - GND commun entre STM32 et alimentation AX12 ?")
    print("     - Un seul fil DATA (half-duplex), pas TX/RX separes")
    print()
    print("  %s2. ALIMENTATION%s" % (BOLD, RST))
    print("     - AX12 alimente en 9-12V (PAS en 3.3V/5V) ?")
    print("     - LED de l'AX12 clignote au power-on ?")
    print("     - Alimentation capable de fournir ~1A ?")
    print()
    print("  %s3. BAUD RATE%s" % (BOLD, RST))
    print("     - Defaut usine AX12 = %s1 000 000 bps%s (1 Mbps)" % (BOLD + YLW, RST))
    print("     - Le firmware doit utiliser AX12_BAUDRATE = 1000000")
    print("     - Si servo reconfigure : adapter robot_config.h")
    print()
    print("  %s4. ID SERVO%s" % (BOLD, RST))
    print("     - Defaut usine = 1 (correspond a AX12_ID_PRISM)")
    print("     - Verifier avec un outil Dynamixel si modifie")
    print()
    print("  %s5. FIRMWARE%s" % (BOLD, RST))
    print("     - Firmware recompile et reflashe apres corrections ?")
    print("     - AX12_Init() appele dans main() ?")
    print("     - Torque Enable envoye au demarrage ?")
    print()

# ============================================================================
# MAIN
# ============================================================================

_ser = None

def sigint_handler(signum, frame_):
    print()
    warn("Ctrl+C -- arret")
    if _ser is not None:
        try:
            _ser.close()
        except Exception:
            pass
    sys.exit(130)

def main():
    global _ser

    parser = argparse.ArgumentParser(
        description="Test et diagnostic servo AX12 (deploy / retract)")
    parser.add_argument("port", nargs="?", default="/dev/serial0",
                        help="Port serie (ex: COM6, /dev/serial0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate liaison STM32 (defaut: 115200)")
    parser.add_argument("--cycles", type=int, default=1,
                        help="Nombre de cycles deploy/retract (defaut: 1)")
    parser.add_argument("--pause", type=float, default=2.0,
                        help="Pause entre deploy et retract (defaut: 2.0 s)")

    args = parser.parse_args()

    print()
    sep()
    print("  %sTEST AX12 -- servo prisme (UART5 / PC12)%s" % (BOLD + CYN, RST))
    sep()
    info("Port STM32  : %s @ %d baud" % (args.port, args.baud))
    info("AX12 config : ID=%d  DEPLOY=%d  RETRACT=%d" % (1, 700, 200))
    info("Cycles      : %d   Pause : %.1f s" % (args.cycles, args.pause))
    sep()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        err("Impossible d'ouvrir %s : %s" % (args.port, e))
        sys.exit(1)

    _ser = ser
    signal.signal(signal.SIGINT, sigint_handler)

    time.sleep(0.3)
    ser.reset_input_buffer()

    # ── 1. Ping STM32 + STATUS ──
    info("Ping STM32 (GET_STATUS)...")
    r = cmd_get_status(ser)
    if r is None:
        err("Pas de reponse du STM32. Verifier le port et le firmware.")
        ser.close()
        sys.exit(2)

    resp, data = r
    if resp == RESP_STATUS:
        ok("STM32 repond (STATUS %d octets)" % len(data))
        st = parse_status(data)
        if st:
            show_status(st)
    else:
        ok("STM32 repond (0x%02X)" % resp)

    # ── 2. Cycles deploy/retract ──
    all_deploy_ok = True
    all_retract_ok = True
    any_ax12_err = False

    for i in range(args.cycles):
        sep()
        info("Cycle %d/%d" % (i + 1, args.cycles))

        # Deploy
        info("DEPLOY_PRISM -> position 700 (~206 deg)...")
        r, dt = cmd_deploy(ser)
        dep_ok, ax12_err = check_response(r, dt, "DEPLOY")
        all_deploy_ok = all_deploy_ok and dep_ok
        any_ax12_err = any_ax12_err or ax12_err

        info("Attente %.1f s (le servo devrait bouger maintenant)..." % args.pause)
        time.sleep(args.pause)

        # Retract
        info("RETRACT_PRISM -> position 200 (~59 deg)...")
        r, dt = cmd_retract(ser)
        ret_ok, ax12_err = check_response(r, dt, "RETRACT")
        all_retract_ok = all_retract_ok and ret_ok
        any_ax12_err = any_ax12_err or ax12_err

        if i < args.cycles - 1:
            info("Attente %.1f s avant prochain cycle..." % args.pause)
            time.sleep(args.pause)

    # ── 3. Recap ──
    sep()
    print()
    print("  %sRECAPITULATIF%s" % (BOLD + CYN, RST))
    sep()

    if any_ax12_err:
        err("ERR_AX12 : le STM32 n'arrive pas a transmettre sur UART5")
        print_checklist()

    elif all_deploy_ok and all_retract_ok:
        ok("Le STM32 a envoye toutes les commandes avec succes (ACK)")
        print()
        print("  %sATTENTION :%s ACK signifie que le STM32 a transmis les" % (BOLD + YLW, RST))
        print("  octets sur UART5. Si le servo n'a %sPAS BOUGE%s :" % (BOLD + RED, RST))
        print()
        print("  %s>> Le probleme est entre UART5 et l'AX12 <<%s" % (BOLD + YLW, RST))
        print()
        print_checklist()

    elif all_deploy_ok or all_retract_ok:
        warn("Reponse partielle -- une commande a echoue")
        print_checklist()

    else:
        err("Aucune commande n'a recu d'ACK")
        print_checklist()

    sep()
    ser.close()
    ok("Port ferme. Fin.")
    print()

if __name__ == "__main__":
    main()
