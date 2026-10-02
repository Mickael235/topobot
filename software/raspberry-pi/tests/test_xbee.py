"""
test_xbee.py — Diagnostic liaison XBee <-> Station Topcon MS1AX

Protocole calque sur ms1ax.py (qui fonctionne) :
  - Terminaison \r\n
  - ACK = \x06 apres chaque commande de config
  - Reponse mesure contient '*ST2' suivi de champs CSV
  - timeout serial = 1.0s

Usage sur Raspberry Pi :
    python3 test_xbee.py
"""

import serial
import serial.tools.list_ports
import time
import sys
import subprocess

# ============================================================
#  CONFIGURATION
# ============================================================
TOPCON_PORT = "/dev/serial0"
TOPCON_BAUD = 9600

# ============================================================
#  UTILITAIRES
# ============================================================

def separator(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")


def list_serial_ports():
    separator("PORTS SERIE DISPONIBLES")
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("[WARN] Aucun port serie detecte !")
        return
    for p in ports:
        print(f"  {p.device}  |  {p.description}  |  hwid={p.hwid}")
    print(f"\n  Total : {len(ports)} port(s)")


def open_port():
    print(f"\n[INFO] Ouverture de {TOPCON_PORT} a {TOPCON_BAUD} baud...")
    try:
        ser = serial.Serial(TOPCON_PORT, TOPCON_BAUD, timeout=1.0)
        ser.dtr = True
        ser.rts = True
        print(f"[OK]   Port ouvert : {ser.name}")
        print(f"       Baudrate={ser.baudrate}  Bytesize={ser.bytesize}  Parity={ser.parity}  Stopbits={ser.stopbits}")
        print(f"       Timeout={ser.timeout}s  DTR={ser.dtr}  RTS={ser.rts}  DSR={ser.dsr}  CTS={ser.cts}")
        return ser
    except serial.SerialException as e:
        print(f"[ERREUR] Impossible d'ouvrir {TOPCON_PORT} : {e}")
        return None


def readport(ser, wait_str, max_tries=15):
    """Lecture identique a ms1ax.py : attend une ligne contenant wait_str.
    Retourne la ligne ou '' si timeout.
    """
    print(f"[RECV]   Attente de {repr(wait_str)} (max {max_tries}s)...")
    for i in range(max_tries):
        raw = ser.readline()
        if raw:
            text = raw.decode("utf-8", errors="ignore").rstrip()
            hex_dump = " ".join(f"{b:02X}" for b in raw[:60])
            print(f"  [{i+1:2d}s] Recu {len(raw)} octets : {repr(text)}")
            print(f"         HEX: {hex_dump}")
            if wait_str in text:
                print(f"[OK]     Trouve {repr(wait_str)} dans la reponse")
                return text
        else:
            elapsed = i + 1
            if elapsed % 5 == 0:
                print(f"  [{elapsed:2d}s] ... rien")
        time.sleep(1)

    print(f"[TIMEOUT] {repr(wait_str)} jamais recu apres {max_tries}s")
    return ""


def send_and_ack(ser, cmd_str, label):
    """Envoie une commande et attend ACK (\\x06) comme ms1ax.py."""
    cmd_bytes = str.encode(cmd_str)
    print(f"\n[ENVOI]  {label}")
    print(f"         Commande : {repr(cmd_str)}")
    ser.write(cmd_bytes)
    ser.flush()
    result = readport(ser, '\x06', max_tries=10)
    if result:
        print(f"[OK]     ACK recu pour {label}")
    else:
        print(f"[WARN]   Pas de ACK pour {label}")
    return result


# ============================================================
#  TEST 1 : SCAN DES PORTS
# ============================================================
def test_scan_ports():
    list_serial_ports()


# ============================================================
#  TEST 2 : OUVERTURE DU PORT
# ============================================================
def test_open_close():
    separator("TEST OUVERTURE / FERMETURE PORT")
    ser = open_port()
    if ser:
        print("[OK]   Fermeture...")
        ser.close()
        print("[OK]   Port ferme")
    return ser is not None


# ============================================================
#  TEST 3 : ECOUTE PASSIVE (la station envoie-t-elle qq chose ?)
# ============================================================
def test_passive_listen():
    separator("TEST ECOUTE PASSIVE (10s)")
    print("On ecoute sans rien envoyer pour voir si la station emet spontanement...")
    ser = open_port()
    if not ser:
        return
    time.sleep(0.5)

    t_start = time.time()
    count = 0
    for i in range(10):
        raw = ser.readline()
        if raw:
            count += 1
            text = raw.decode("utf-8", errors="ignore").rstrip()
            hex_dump = " ".join(f"{b:02X}" for b in raw[:40])
            print(f"  [{i+1}s] Recu : {repr(text)}")
            print(f"        HEX: {hex_dump}")
        else:
            print(f"  [{i+1}s] rien")
        time.sleep(1)

    if count == 0:
        print("[INFO] Aucune emission spontanee detectee (normal)")
    ser.close()


# ============================================================
#  TEST 4 : ALLUMAGE LASER (*GLON)
# ============================================================
def test_laser_on():
    separator("TEST LASER ON (*GLON)")
    ser = open_port()
    if not ser:
        return
    time.sleep(1)

    print("\n[ENVOI] *GLON")
    ser.write(str.encode("*GLON\r\n"))
    ser.flush()
    readport(ser, '\x06', max_tries=5)

    print("\n[INFO] Le laser devrait etre ALLUME sur la station.")
    print("       Regardez physiquement si le point laser est visible.")
    print("       Appuyez sur Entree pour continuer...")

    try:
        input()
    except EOFError:
        time.sleep(3)

    ser.close()


# ============================================================
#  TEST 5 : EXTINCTION LASER (*GLOFF)
# ============================================================
def test_laser_off():
    separator("TEST LASER OFF (*GLOFF)")
    ser = open_port()
    if not ser:
        return
    time.sleep(1)

    print("\n[ENVOI] *GLOFF")
    ser.write(str.encode("*GLOFF\r\n"))
    ser.flush()
    readport(ser, '\x06', max_tries=5)

    print("[INFO] Le laser devrait etre ETEINT.")
    ser.close()


# ============================================================
#  TEST 6 : MODE STANDARD (identique a ms1ax.py mode_standard)
# ============================================================
def test_standard_mode():
    separator("TEST MODE STANDARD (ms1ax.py)")
    ser = open_port()
    if not ser:
        return
    time.sleep(1)

    send_and_ack(ser, "*/PA 1,0,, \r\n", "PA — mode search")
    send_and_ack(ser, "*/PH 0 \r\n",     "PH — pointe precis")
    send_and_ack(ser, "Xa\r\n",           "Xa — Fine S mode")

    print("\n[OK] Mode standard configure")
    ser.close()


# ============================================================
#  TEST 7 : MESURE COMPLETE (identique a ms1ax.py do_measure)
# ============================================================
def test_full_measure():
    separator("TEST MESURE COMPLETE (protocole ms1ax.py)")
    ser = open_port()
    if not ser:
        return
    time.sleep(1)

    # Etape 1 : Mode standard (comme ms1ax.py)
    print("\n--- Etape 1/4 : Mode standard ---")
    send_and_ack(ser, "*/PA 1,0,, \r\n", "PA")
    send_and_ack(ser, "*/PH 0 \r\n",     "PH")
    send_and_ack(ser, "Xa\r\n",           "Xa")

    # Etape 2 : Laser ON
    print("\n--- Etape 2/4 : Laser ON ---")
    ser.write(str.encode("*GLON\r\n"))
    ser.flush()
    readport(ser, '\x06', max_tries=5)
    time.sleep(1.5)

    # Etape 3 : Mesure (attend reponse contenant *ST2)
    print("\n--- Etape 3/4 : Mesure (*ST2) ---")
    ser.write(str.encode("*ST2\r\n"))
    ser.flush()
    result = readport(ser, '*ST2', max_tries=20)

    # Etape 4 : Laser OFF
    print("\n--- Etape 4/4 : Laser OFF ---")
    ser.write(str.encode("*GLOFF\r\n"))
    ser.flush()
    time.sleep(0.5)

    # Parsing
    print("\n--- Parsing ---")
    if not result:
        print("[ERREUR] Pas de trame *ST2 recue")
        ser.close()
        return 0.0

    tab = result.split(',')
    print(f"[PARSE]  Champs ({len(tab)}) : {tab}")

    if len(tab) >= 5:
        try:
            hz = tab[2]
            v = tab[3]
            d = float(tab[4])
            print(f"[OK]     Hz={hz} gon  V={v} gon  D={d:.3f} m")
        except (ValueError, IndexError) as e:
            print(f"[ERREUR] Parsing : {e}")
            d = 0.0
    else:
        print(f"[ERREUR] Pas assez de champs ({len(tab)}), attendu >= 5")
        d = 0.0

    ser.close()
    return d


# ============================================================
#  TEST 8 : ENVOI BRUT + DUMP HEX (debug bas niveau)
# ============================================================
def test_raw_echo():
    separator("TEST ENVOI BRUT + DUMP RAW")
    ser = open_port()
    if not ser:
        return
    time.sleep(1)

    print("\n[ENVOI] Retour chariot seul")
    ser.write(b"\r\n")
    ser.flush()
    readport(ser, '', max_tries=3)

    print("\n[ENVOI] Asterisque seul")
    ser.write(b"*\r\n")
    ser.flush()
    readport(ser, '', max_tries=3)

    ser.close()


# ============================================================
#  TEST 9 : 3 MESURES D'AFFILEE (meme port, sans fermer)
# ============================================================
def test_repeated_measures():
    separator("TEST 3 MESURES REPETEES (meme port)")
    ser = open_port()
    if not ser:
        return
    time.sleep(1)

    # Init mode standard une seule fois
    print("\n--- Init mode standard ---")
    send_and_ack(ser, "*/PA 1,0,, \r\n", "PA")
    send_and_ack(ser, "*/PH 0 \r\n",     "PH")
    send_and_ack(ser, "Xa\r\n",           "Xa")

    for i in range(1, 4):
        print(f"\n{'*'*50}")
        print(f"  MESURE {i}/3")
        print(f"{'*'*50}")

        # Laser ON
        print(f"[{i}] Laser ON...")
        ser.write(str.encode("*GLON\r\n"))
        ser.flush()
        readport(ser, '\x06', max_tries=5)
        time.sleep(1.5)

        # Mesure
        print(f"[{i}] Mesure *ST2...")
        ser.write(str.encode("*ST2\r\n"))
        ser.flush()
        result = readport(ser, '*ST2', max_tries=20)

        # Laser OFF
        print(f"[{i}] Laser OFF...")
        ser.write(str.encode("*GLOFF\r\n"))
        ser.flush()
        time.sleep(0.5)
        ser.flushInput()

        # Parsing
        if result:
            tab = result.split(',')
            if len(tab) >= 5:
                try:
                    print(f"[{i}] RESULTAT : Hz={tab[2]} V={tab[3]} D={float(tab[4]):.3f} m")
                except ValueError:
                    print(f"[{i}] RESULTAT : parsing erreur sur {tab}")
            else:
                print(f"[{i}] RESULTAT : {len(tab)} champs seulement")
        else:
            print(f"[{i}] RESULTAT : ECHEC (pas de reponse *ST2)")

        time.sleep(2)

    ser.close()
    print("\n[OK] Test mesures repetees termine")


# ============================================================
#  TEST 10 : DIAGNOSTIC RX (serial console, loopback, cablage)
# ============================================================
def test_diag_rx():
    separator("DIAGNOSTIC RECEPTION (RX)")

    # 1. Verifier si le serial console est actif (il mange les octets RX)
    print("\n--- 1. Serial console (getty) ---")
    try:
        r = subprocess.run(
            ["systemctl", "is-active", "serial-getty@ttyS0.service"],
            capture_output=True, text=True, timeout=5
        )
        status = r.stdout.strip()
        print(f"  serial-getty@ttyS0 : {status}")
        if status == "active":
            print("  [PROBLEME] Le serial console est ACTIF !")
            print("  Il capture les octets entrants sur serial0.")
            print("  Pour le desactiver :")
            print("    sudo raspi-config -> Interface Options -> Serial Port")
            print("    -> Login shell over serial : NON")
            print("    -> Serial hardware enabled  : OUI")
            print("    Puis reboot")
        else:
            print("  [OK] Serial console desactive")
    except Exception as e:
        print(f"  Impossible de verifier : {e}")

    # 2. Verifier cmdline.txt
    print("\n--- 2. /boot/firmware/cmdline.txt ---")
    for path in ["/boot/firmware/cmdline.txt", "/boot/cmdline.txt"]:
        try:
            r = subprocess.run(
                ["cat", path],
                capture_output=True, text=True, timeout=5
            )
            if r.returncode == 0 and "DO NOT EDIT" not in r.stdout:
                cmdline = r.stdout.strip()
                print(f"  ({path})")
                print(f"  {cmdline}")
                if "console=serial0" in cmdline or "console=ttyAMA0" in cmdline:
                    print("  [PROBLEME] Console serie active dans cmdline.txt !")
                else:
                    print("  [OK] Pas de console serie")
                break
        except Exception:
            pass

    # 2b. Verifier config.txt pour UART/Bluetooth
    print("\n--- 2b. Config UART/Bluetooth ---")
    for path in ["/boot/firmware/config.txt", "/boot/config.txt"]:
        try:
            r = subprocess.run(
                ["cat", path],
                capture_output=True, text=True, timeout=5
            )
            if r.returncode == 0:
                print(f"  ({path})")
                for line in r.stdout.splitlines():
                    low = line.lower()
                    if any(k in low for k in ["uart", "serial", "miniuart", "disable-bt", "ttyama"]):
                        print(f"  {line}")
                break
        except Exception:
            pass

    # 3. Verifier a quoi serial0 pointe
    print("\n--- 3. Lien symbolique /dev/serial0 ---")
    try:
        r = subprocess.run(
            ["ls", "-la", "/dev/serial0"],
            capture_output=True, text=True, timeout=5
        )
        link = r.stdout.strip()
        print(f"  {link}")
        if "ttyS0" in link:
            print("  [ATTENTION] serial0 -> ttyS0 = MINI UART")
            print("  Le mini UART est INSTABLE : son baud rate varie")
            print("  avec la frequence CPU. C'est probablement la cause")
            print("  du probleme (marche une fois puis plus).")
            print("  ")
            print("  SOLUTION : ajouter dans /boot/firmware/config.txt :")
            print("    dtoverlay=miniuart-bt")
            print("  ou pour desactiver le bluetooth :")
            print("    dtoverlay=disable-bt")
            print("  puis reboot. serial0 pointera vers ttyAMA0 (UART fiable).")
        elif "ttyAMA0" in link:
            print("  [OK] serial0 -> ttyAMA0 = PL011 UART (fiable)")
    except Exception as e:
        print(f"  {e}")

    # 4. Verifier si un autre process utilise le port
    print("\n--- 4. Processus utilisant serial0/ttyS0/ttyAMA0 ---")
    try:
        r = subprocess.run(
            ["sudo", "fuser", "/dev/serial0"],
            capture_output=True, text=True, timeout=5
        )
        pids = r.stdout.strip()
        if pids:
            print(f"  PIDs utilisant /dev/serial0 : {pids}")
            print("  [WARN] Un autre processus utilise le port !")
            for pid in pids.split():
                r2 = subprocess.run(
                    ["ps", "-p", pid.strip(), "-o", "comm="],
                    capture_output=True, text=True, timeout=5
                )
                print(f"    PID {pid.strip()} = {r2.stdout.strip()}")
        else:
            print("  [OK] Aucun processus ne bloque le port")
    except Exception as e:
        print(f"  {e}")

    # 5. Test loopback (connecter TX sur RX physiquement)
    print("\n--- 5. Test loopback (TX -> RX) ---")
    print("  Si vous connectez le fil TX du Pi au fil RX du Pi")
    print("  (sans le XBee), on devrait recevoir ce qu'on envoie.")
    print("  Voulez-vous tester le loopback ? (o/n)")
    try:
        ans = input("  > ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        ans = "n"

    if ans == "o":
        ser = open_port()
        if ser:
            test_msg = b"LOOPBACK_TEST_123\r\n"
            print(f"  Envoi : {repr(test_msg)}")
            ser.reset_input_buffer()
            ser.write(test_msg)
            ser.flush()
            time.sleep(0.5)
            if ser.in_waiting > 0:
                raw = ser.read(ser.in_waiting)
                text = raw.decode("utf-8", errors="ignore").strip()
                print(f"  Recu  : {repr(text)}")
                if "LOOPBACK_TEST_123" in text:
                    print("  [OK] Loopback fonctionne ! RX du Pi est OK.")
                    print("  Le probleme est donc cote XBee/station.")
                else:
                    print(f"  [WARN] Recu quelque chose mais pas le bon contenu")
            else:
                print("  [ERREUR] Rien recu ! Le RX du Pi ne fonctionne pas.")
                print("  Verifier le cablage GPIO14(TX)/GPIO15(RX).")
            ser.close()

    print("\n--- Resume ---")
    print("  Si le serial console est actif  -> desactiver via raspi-config")
    print("  Si loopback echoue              -> probleme cablage RX du Pi")
    print("  Si loopback OK mais station non -> probleme XBee retour ou config station")


# ============================================================
#  MENU PRINCIPAL
# ============================================================
def main():
    print("""
============================================================
   TopoBot — Diagnostic XBee / Station Topcon MS1AX
============================================================
   Port : {port}
   Baud : {baud}
============================================================
""".format(port=TOPCON_PORT, baud=TOPCON_BAUD))

    tests = [
        ("Scan ports serie",         test_scan_ports),
        ("Ouverture / fermeture",    test_open_close),
        ("Ecoute passive (10s)",     test_passive_listen),
        ("Laser ON  (*GLON)",        test_laser_on),
        ("Laser OFF (*GLOFF)",       test_laser_off),
        ("Mode standard (PA/PH/Xa)", test_standard_mode),
        ("Mesure complete (*ST2)",   test_full_measure),
        ("Envoi brut (debug)",       test_raw_echo),
        ("3 mesures repetees",       test_repeated_measures),
        ("DIAGNOSTIC RX",            test_diag_rx),
    ]

    if len(sys.argv) > 1:
        try:
            choice = int(sys.argv[1])
            if 1 <= choice <= len(tests):
                label, func = tests[choice - 1]
                print(f">>> Execution du test {choice} : {label}")
                func()
                return
        except ValueError:
            pass

    print("Tests disponibles :")
    for i, (label, _) in enumerate(tests, 1):
        print(f"  {i}. {label}")
    print(f"  0. Executer TOUS les tests")
    print(f"  q. Quitter")

    while True:
        try:
            choice = input("\nChoix > ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nAu revoir.")
            break

        if choice == "q":
            break

        if choice == "0":
            for i, (label, func) in enumerate(tests, 1):
                print(f"\n>>> Test {i}/{len(tests)} : {label}")
                try:
                    func()
                except Exception as e:
                    print(f"[EXCEPTION] {e}")
            print("\n>>> Tous les tests termines.")
            continue

        try:
            idx = int(choice) - 1
            if 0 <= idx < len(tests):
                label, func = tests[idx]
                print(f"\n>>> {label}")
                try:
                    func()
                except Exception as e:
                    print(f"[EXCEPTION] {e}")
            else:
                print("Numero invalide")
        except ValueError:
            print("Entrez un numero (1-10), 0 pour tout, q pour quitter")


if __name__ == "__main__":
    main()
