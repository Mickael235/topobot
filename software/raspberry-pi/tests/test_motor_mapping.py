#!/usr/bin/env python3
# -*- coding: ascii -*-

"""
test_motor_mapping.py - Diagnostic mapping moteur Sabertooth / encodeur

Utilise SET_VELOCITY avec des combinaisons vx/vy qui ISOLENT des paires
de roues grace a la cinematique mecanum :

  w_FL = (vx - vy) / r    w_RR = (vx - vy) / r
  w_FR = (vx + vy) / r    w_RL = (vx + vy) / r

Test A : vx = +v, vy = +v  =>  w_FL = 0, w_RR = 0  (seuls FR et RL tournent)
Test B : vx = +v, vy = -v  =>  w_FR = 0, w_RL = 0  (seuls FL et RR tournent)
Test C : vx = +v, vy = 0   =>  les 4 roues au meme rythme (verifie direction)

Si Test A montre FL+RR qui tournent au lieu de FR+RL :
  => les canaux moteur A et B sont INVERSES sur les Sabertooth.

IMPORTANT : sureleveR le robot ou le placer sur une surface degagee
            avant de lancer ce test !

Usage :
  python3 test_motor_mapping.py /dev/serial0
  python3 test_motor_mapping.py /dev/serial0 --speed 0.08 --duration 1.5
"""

import argparse
import serial
import struct
import sys
import time

# ============================================================================
# PROTOCOLE (identique a test_move_to.py)
# ============================================================================

PROTO_START      = 0xAA
CMD_STOP         = 0x02
CMD_RESET_ODOM   = 0x03
CMD_GET_STATUS   = 0x04
CMD_SET_VELOCITY = 0x07

RESP_ACK    = 0xA1
RESP_STATUS = 0xA3

STATUS_FMT  = "<fffiiiiffffffB"
STATUS_SIZE = struct.calcsize(STATUS_FMT)   # 53

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
MAG  = "\033[95m"

# ============================================================================
# COMMUNICATION
# ============================================================================

def crc(cmd, payload):
    c = cmd ^ len(payload)
    for b in payload:
        c ^= b
    return c & 0xFF


def send(ser, cmd, payload=b""):
    f = bytes([PROTO_START, cmd, len(payload)]) + payload + bytes([crc(cmd, payload)])
    ser.write(f)
    ser.flush()


def recv(ser, timeout=0.5):
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
        if crc_rx == crc(resp, data):
            return resp, data
    return None


def cmd_stop(ser):
    for _ in range(3):
        ser.reset_input_buffer()
        send(ser, CMD_STOP)
        r = recv(ser, 0.3)
        if r and r[0] == RESP_ACK:
            return True
        time.sleep(0.05)
    return False


def cmd_reset_odom(ser):
    send(ser, CMD_RESET_ODOM)
    return recv(ser, 0.5)


def cmd_set_velocity(ser, vx, vy, wz=0.0):
    payload = struct.pack("<fff", vx, vy, wz)
    send(ser, CMD_SET_VELOCITY, payload)
    return recv(ser, 0.5)


def cmd_get_status(ser):
    send(ser, CMD_GET_STATUS)
    r = recv(ser, 0.4)
    if r is None:
        return None
    resp, data = r
    if resp != RESP_STATUS or len(data) < STATUS_SIZE:
        return None
    f = struct.unpack_from(STATUS_FMT, data, 0)
    return dict(e_FL=f[3], e_FR=f[4], e_RL=f[5], e_RR=f[6],
                x=f[7], y=f[8], theta=f[9])


def signed_delta_16(curr, prev):
    d = (int(curr) - int(prev)) & 0xFFFF
    return d - 65536 if d >= 32768 else d


# ============================================================================
# TESTS
# ============================================================================

THRESHOLD = 30   # ticks minimum pour considerer qu'une roue a tourne

WHEELS = ["FL", "FR", "RL", "RR"]


def read_encoders(ser):
    """Retourne (e_FL, e_FR, e_RL, e_RR) ou None."""
    st = cmd_get_status(ser)
    if st is None:
        return None
    return (st["e_FL"], st["e_FR"], st["e_RL"], st["e_RR"])


def run_test(ser, label, vx, vy, duration, expected_moving, expected_still):
    """
    Envoie SET_VELOCITY(vx, vy) pendant `duration` secondes, lit les deltas.
    expected_moving / expected_still : listes de noms de roue.
    """
    print()
    print("  %s%s%s" % (BOLD + CYN, "=" * 56, RST))
    print("  %s  %s%s" % (BOLD, label, RST))
    print("  %s  vx = %+.3f   vy = %+.3f   duree = %.1f s%s" % (DIM, vx, vy, duration, RST))
    print("  %s  Attendu : %s tournent, %s immobiles%s" % (
        DIM, "+".join(expected_moving), "+".join(expected_still), RST))
    print("  %s%s%s" % (BOLD + CYN, "=" * 56, RST))

    enc_before = read_encoders(ser)
    if enc_before is None:
        print("  %sERREUR: pas de reponse STATUS avant test%s" % (RED, RST))
        return None

    cmd_set_velocity(ser, vx, vy)
    print("  Commande envoyee... ", end="", flush=True)

    # Polling rapide pour voir l'evolution
    t0 = time.time()
    while time.time() - t0 < duration:
        remaining = duration - (time.time() - t0)
        print("\r  En cours... %.1f s restantes   " % remaining, end="", flush=True)
        time.sleep(0.2)
    print("\r  Duree ecoulee, STOP.                    ")

    cmd_stop(ser)
    time.sleep(0.3)

    enc_after = read_encoders(ser)
    if enc_after is None:
        print("  %sERREUR: pas de reponse STATUS apres test%s" % (RED, RST))
        return None

    deltas = {}
    for i, w in enumerate(WHEELS):
        deltas[w] = signed_delta_16(enc_after[i], enc_before[i])

    # Affichage
    print()
    print("  %sDeltas encodeurs bruts (ticks) :%s" % (BOLD, RST))
    for w in WHEELS:
        d = deltas[w]
        moved = abs(d) > THRESHOLD
        if w in expected_moving:
            col = GRN if moved else RED
            status = "TOURNE (attendu)" if moved else "IMMOBILE (ANORMAL!)"
        else:
            col = GRN if not moved else YLW + BOLD
            status = "immobile (attendu)" if not moved else "TOURNE (INATTENDU!)"
        print("    %s : %s%+7d%s   %s%s%s" % (w, BOLD, d, RST, col, status, RST))

    # Verdict
    actually_moving = [w for w in WHEELS if abs(deltas[w]) > THRESHOLD]
    actually_still  = [w for w in WHEELS if abs(deltas[w]) <= THRESHOLD]

    ok = (set(actually_moving) == set(expected_moving))
    if ok:
        print("  %s=> RESULTAT CORRECT%s" % (GRN + BOLD, RST))
    else:
        # Check for A/B swap pattern
        swapped_moving = set(expected_still)  # if swapped, the "still" ones move
        if set(actually_moving) == swapped_moving:
            print("  %s=> SWAP DETECTE : les roues opposees tournent !%s" % (RED + BOLD, RST))
        else:
            print("  %s=> RESULTAT INATTENDU : verifier cablage%s" % (YLW + BOLD, RST))
        print("  %s   Ont tourne  : %s%s" % (DIM, ", ".join(actually_moving) or "(aucune)", RST))
        print("  %s   Immobiles   : %s%s" % (DIM, ", ".join(actually_still) or "(aucune)", RST))

    return deltas


def analyze_direction(deltas_c):
    """Analyse le test C (4 roues avant) pour verifier les directions."""
    if deltas_c is None:
        return

    print()
    print("  %s%s%s" % (BOLD + MAG, "=" * 56, RST))
    print("  %s  ANALYSE DIRECTION (Test C - toutes les roues)%s" % (BOLD, RST))
    print("  %s%s%s" % (BOLD + MAG, "=" * 56, RST))
    print()

    # Encodeurs gauche : A/B swappe => delta brut NEGATIF = roue avance
    # Encodeurs droite : A/B normal => delta brut POSITIF = roue avance
    # (C'est l'hypothese basee sur le cablage CubeMX)
    expected_sign = {
        "FL": "negatif (A/B swappe, TIM8)",
        "FR": "positif (A/B normal, TIM3)",
        "RL": "negatif (A/B swappe, TIM4)",
        "RR": "positif (A/B normal, TIM2)",
    }

    for w in WHEELS:
        d = deltas_c[w]
        if abs(d) <= THRESHOLD:
            print("    %s : delta=%+7d  -- IMMOBILE (probleme !)" % (w, d))
            continue

        print("    %s : delta = %s%+7d%s  (attendu %s)" % (
            w, BOLD, d, RST, expected_sign[w]))

    # Coherence des magnitudes
    mags = [abs(deltas_c[w]) for w in WHEELS if abs(deltas_c[w]) > THRESHOLD]
    if len(mags) >= 2:
        avg = sum(mags) / len(mags)
        print()
        print("  Magnitudes : %s" % "  ".join(
            "%s=%d" % (w, abs(deltas_c[w])) for w in WHEELS))
        if avg > 0:
            deviations = [(abs(deltas_c[w]) - avg) / avg * 100 for w in WHEELS]
            max_dev = max(abs(d) for d in deviations)
            col = GRN if max_dev < 20 else (YLW if max_dev < 40 else RED)
            print("  Ecart max vs moyenne : %s%.0f%%%s" % (col, max_dev, RST))

    # Verification des signes : gauche negatif, droite positif ?
    fl_sign_ok = deltas_c["FL"] < -THRESHOLD
    fr_sign_ok = deltas_c["FR"] > THRESHOLD
    rl_sign_ok = deltas_c["RL"] < -THRESHOLD
    rr_sign_ok = deltas_c["RR"] > THRESHOLD

    all_signs_ok = fl_sign_ok and fr_sign_ok and rl_sign_ok and rr_sign_ok
    if all_signs_ok:
        print()
        print("  %sDIRECTIONS OK : toutes les roues avancent dans le bon sens%s" % (
            GRN + BOLD, RST))
        print("  %s  => Les POL_xx (moteur) et ENC_POL_xx (encodeur) sont coherents%s" % (
            DIM, RST))
    else:
        print()
        print("  %sPROBLEME DE DIRECTION detecte :%s" % (RED + BOLD, RST))
        for w, ok_val, d in [("FL", fl_sign_ok, deltas_c["FL"]),
                              ("FR", fr_sign_ok, deltas_c["FR"]),
                              ("RL", rl_sign_ok, deltas_c["RL"]),
                              ("RR", rr_sign_ok, deltas_c["RR"])]:
            if not ok_val and abs(d) > THRESHOLD:
                print("    %s%s : signe INVERSE (delta=%+d, attendu %s)%s" % (
                    RED, w, d,
                    "negatif" if w in ("FL", "RL") else "positif",
                    RST))
                if w in ("FL", "RL"):
                    print("      => Inverser POL_%s dans mecanum.c OU ENC_POL_%s dans robot_config.h" % (w, w))
                else:
                    print("      => Inverser POL_%s dans mecanum.c OU ENC_POL_%s dans robot_config.h" % (w, w))


def main():
    parser = argparse.ArgumentParser(
        description="Diagnostic mapping moteur/encodeur par isolation de paires de roues")
    parser.add_argument("port", nargs="?", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--speed", type=float, default=0.06,
                        help="Vitesse de test (m/s, defaut 0.06)")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Duree de chaque test en secondes (defaut 2.0)")
    args = parser.parse_args()

    v = args.speed
    dur = args.duration

    print()
    print("%s%s%s" % (BOLD + CYN, "=" * 60, RST))
    print("  %sDIAGNOSTIC MAPPING MOTEUR SABERTOOTH / ENCODEUR%s" % (BOLD, RST))
    print("%s%s%s" % (BOLD + CYN, "=" * 60, RST))
    print()
    print("  %s!!! IMPORTANT : surelever le robot ou le placer sur%s" % (RED + BOLD, RST))
    print("  %s!!!             une surface degagee avant de continuer%s" % (RED + BOLD, RST))
    print()
    print("  Ce test va envoyer 3 patrons de vitesse (%.1f s chacun)" % dur)
    print("  pour identifier quel canal Sabertooth entraine quelle roue.")
    print()
    print("  Cinematique mecanum :")
    print("    w_FL = (vx - vy) / r     w_RR = (vx - vy) / r")
    print("    w_FR = (vx + vy) / r     w_RL = (vx + vy) / r")
    print()
    print("  Test A : vx=+v, vy=+v => FL et RR a ZERO (isole FR+RL)")
    print("  Test B : vx=+v, vy=-v => FR et RL a ZERO (isole FL+RR)")
    print("  Test C : vx=+v, vy=0  => 4 roues identiques (verifie direction)")
    print()

    input("  Appuyer sur Entree pour demarrer (Ctrl+C pour annuler)...")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print("%sERREUR: %s%s" % (RED, e, RST))
        sys.exit(1)

    time.sleep(0.3)
    ser.reset_input_buffer()

    # Ping
    st = cmd_get_status(ser)
    if st is None:
        print("%sERREUR: STM32 ne repond pas. Verifier firmware et port.%s" % (RED, RST))
        ser.close()
        sys.exit(1)
    print("  %sSTM32 connecte OK%s" % (GRN, RST))

    # Stop + Reset
    cmd_stop(ser)
    time.sleep(0.3)
    cmd_reset_odom(ser)
    time.sleep(0.5)

    # ---------------------------------------------------------------
    # Test A : vx=+v, vy=+v => seuls FR et RL devraient tourner
    # ---------------------------------------------------------------
    deltas_a = run_test(ser, "Test A : isole FR + RL  (FL=RR=0)",
                        vx=v, vy=v, duration=dur,
                        expected_moving=["FR", "RL"],
                        expected_still=["FL", "RR"])
    time.sleep(1.5)
    cmd_reset_odom(ser)
    time.sleep(0.3)

    # ---------------------------------------------------------------
    # Test B : vx=+v, vy=-v => seuls FL et RR devraient tourner
    # ---------------------------------------------------------------
    deltas_b = run_test(ser, "Test B : isole FL + RR  (FR=RL=0)",
                        vx=v, vy=-v, duration=dur,
                        expected_moving=["FL", "RR"],
                        expected_still=["FR", "RL"])
    time.sleep(1.5)
    cmd_reset_odom(ser)
    time.sleep(0.3)

    # ---------------------------------------------------------------
    # Test C : vx=+v, vy=0 => les 4 roues au meme rythme
    # ---------------------------------------------------------------
    deltas_c = run_test(ser, "Test C : 4 roues en avant  (verification direction)",
                        vx=v, vy=0.0, duration=dur,
                        expected_moving=["FL", "FR", "RL", "RR"],
                        expected_still=[])
    time.sleep(0.5)

    # Stop final
    cmd_stop(ser)

    # ---------------------------------------------------------------
    # Analyse direction (Test C)
    # ---------------------------------------------------------------
    analyze_direction(deltas_c)

    # ---------------------------------------------------------------
    # Diagnostic final
    # ---------------------------------------------------------------
    print()
    print("%s%s%s" % (BOLD + CYN, "=" * 60, RST))
    print("  %sDIAGNOSTIC FINAL%s" % (BOLD, RST))
    print("%s%s%s" % (BOLD + CYN, "=" * 60, RST))

    if deltas_a is not None and deltas_b is not None:
        # Determine qui a tourne dans chaque test
        moved_a = set(w for w in WHEELS if abs(deltas_a[w]) > THRESHOLD)
        moved_b = set(w for w in WHEELS if abs(deltas_b[w]) > THRESHOLD)

        mapping_ok = (moved_a == {"FR", "RL"} and moved_b == {"FL", "RR"})
        swap_both  = (moved_a == {"FL", "RR"} and moved_b == {"FR", "RL"})
        swap_s1    = (moved_a - {"FR"} == {"FL"} or moved_b - {"RR"} == {"RL"})

        if mapping_ok:
            print()
            print("  %sMAPPING MOTEUR CORRECT%s" % (GRN + BOLD, RST))
            print("  SABRE1 motor A = FL, motor B = RL  (confirme)")
            print("  SABRE2 motor A = FR, motor B = RR  (confirme)")
            print()
            print("  Si le robot derive quand meme, le probleme est dans la")
            print("  DIRECTION (POL_xx ou ENC_POL_xx). Voir l'analyse ci-dessus.")
        elif swap_both:
            print()
            print("  %sSWAP A/B DETECTE SUR LES DEUX SABERTOOTH !%s" % (RED + BOLD, RST))
            print()
            print("  Mapping REEL :")
            print("    SABRE1 (addr 128) motor A = %sRL%s  (firmware croit FL)" % (RED, RST))
            print("    SABRE1 (addr 128) motor B = %sFL%s  (firmware croit RL)" % (RED, RST))
            print("    SABRE2 (addr 129) motor A = %sRR%s  (firmware croit FR)" % (RED, RST))
            print("    SABRE2 (addr 129) motor B = %sFR%s  (firmware croit RR)" % (RED, RST))
            print()
            print("  %sCORRECTION dans mecanum.c :%s" % (BOLD + YLW, RST))
            print("    Inverser motor 0 <-> 1 pour chaque Sabertooth :")
            print("      FL -> SABx motor B (etait A)")
            print("      RL -> SABx motor A (etait B)")
            print("      FR -> SABx motor B (etait A)")
            print("      RR -> SABx motor A (etait B)")
            print()
            print("  %sET AUSSI inverser POL_FL <-> POL_RL et POL_FR <-> POL_RR%s" % (YLW, RST))
        else:
            print()
            print("  %sMAPPING PARTIEL OU INATTENDU%s" % (YLW + BOLD, RST))
            print("  Test A (attendu FR+RL) : ont tourne = %s" % ", ".join(sorted(moved_a)))
            print("  Test B (attendu FL+RR) : ont tourne = %s" % ", ".join(sorted(moved_b)))
            print("  Verifier le cablage moteur manuellement.")

    print()
    print("%s%s%s" % (DIM, "=" * 60, RST))
    ser.close()
    print("  %sPort ferme. Fin du diagnostic.%s" % (GRN, RST))
    print()


if __name__ == "__main__":
    main()
