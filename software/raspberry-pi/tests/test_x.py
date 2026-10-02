import sys
import serial
import time

# --- CONFIGURATION ---
PORT_DEFAULT = '/dev/serial0'
BAUDRATE = 9600

#--------------------- FONCTIONS ----------------

def readport(port, wait_for, timeout_max=15):
    """ Lit le port série et attend une réponse spécifique ou un ACK. """
    t_end = time.time() + timeout_max
    while time.time() < t_end:
        if port.in_waiting > 0:
            try:
                line = port.readline().decode('utf-8').strip()
                if line:
                    print(f"DEBUG: Station dit -> {line}")
                    if "E" in line and len(line) < 5: 
                        print("... Station en mouvement ...")
                        time.sleep(1)
                        continue
                    if wait_for in line or '\x06' in line:
                        return line
            except Exception as e:
                print(f"Erreur décodage: {e}")
        time.sleep(0.1) 
    return ""

def wake_up_instrument(port):
    """ Procédure de réveil (Page 30 du manuel). """
    print("Tentative de réveil de l'instrument...")
    port.write(b"\r") # Envoi du <Cr> seul
    time.sleep(0.2)   # Délai de 200ms
    port.write(b"*PON\r\n") # Power On
    time.sleep(2)

def mode_standard(port):
    """ Combine le réveil et la configuration standard complète. """
    print("--> Configuration (Mode Standard)...")
    wake_up_instrument(port)
    
    port.write(b"*RM1\r\n") # Mode distant
    time.sleep(0.5)
    
    print("Réglage paramètres standard...")
    port.write(b"*/PA 1,0,,\r\n") # Mode Search Standard
    readport(port, "ACK", timeout_max=5)
    
    port.write(b"*/PH 0\r\n") # Pointé Précis
    readport(port, "ACK", timeout_max=5)
    
    port.write(b"Xa\r\n") # Mode Fine S
    readport(port, "ACK", timeout_max=5)
    print("--> Station prête et verrouillée en mode Standard.")

def do_measure(port):
    """ Mode mesure de la version 11. """
    port.write(b"*GLON\r\n")
    print("Laser ON - Stabilisation visée...")
    time.sleep(2) 
    
    print("Lancement de la mesure (*ST2)...")
    port.reset_input_buffer()
    port.write(b"*ST2\r\n") #
    
    resultat = readport(port, "*ST2", timeout_max=20)
    
    if resultat and ',' in resultat:
        tab = resultat.split(',')
        if len(tab) >= 5:
            try:
                hz, v, dist = tab[2], tab[3], tab[4]
                print(f"\n[MESURE] Hz: {hz} gon | V: {v} gon | D: {dist} m\n")
            except Exception:
                print("Erreur d'interprétation.")
    else:
        print("Erreur : Pas de réponse à la mesure.")
    
    port.write(b"*GLOFF\r\n")

def do_tracking(port, duration=10):
    """ Mode tracking complet de la version 11. """
    print("--> Configuration : Mode Tracking...")
    port.write(b"*/PA 1,1,,\r\n") # Mode poursuite
    time.sleep(0.5)
    port.write(b"*/PH 1\r\n") # Pointé rapide
    time.sleep(0.5)
    port.write(b"Xe\r\n") # Mesure track
    time.sleep(0.5)

    print(f"Flux ST2 pendant {duration}s...")
    port.write(b"*ST2\r\n")
    
    t_end = time.time() + duration
    while time.time() < t_end:
        if port.in_waiting > 0:
            ligne = port.readline().decode('utf-8').strip()
            if ligne: print(f"Flux : {ligne}")
        time.sleep(0.1)

    port.write(b"*ST0\r\n") # Stop
    print("Tracking terminé.")

def fast_relock(port):
    """ Mode relock avec saisie manuelle des valeurs. """
    print("\n--- Mode Relocalisation Rapide (*ST4) ---")
    try:
        hz = input("Entrez l'angle Horizontal (gon) : ")
        v = input("Entrez l'angle Vertical (gon) : ")
        print(f"Relocalisation vers Hz:{hz}, V:{v}...")
        # Format : *ST4 [HAR],[VA],
        cmd = f"*ST4 {hz},{v},\r\n".encode()
        port.write(cmd)
        readport(port, "ACK", timeout_max=5)
    except ValueError:
        print("Erreur : Veuillez entrer des nombres valides.")

# --------- PRINCIPAL --------------

if len(sys.argv) < 2:
    print('Modes : std, mes, track, laser, relock')
    sys.exit()

mode = sys.argv[1]

try:
    ser = serial.Serial(PORT_DEFAULT, baudrate=BAUDRATE, timeout=2)
    time.sleep(2) 

    if mode == 'std':
        mode_standard(ser)
    elif mode == 'mes':
        do_measure(ser)
    elif mode == 'track':
        do_tracking(ser)
    elif mode == 'laser':
        wake_up_instrument(ser)
        print("Laser ON (10s)...")
        ser.write(b"*GLON\r\n")
        time.sleep(10)
        ser.write(b"*GLOFF\r\n")
    elif mode == 'relock':
        fast_relock(ser)

    ser.close()
except Exception as e:
    print(f"Erreur : {e}")