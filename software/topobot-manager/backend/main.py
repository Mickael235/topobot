"""
=============================================================================
  TopoBot Manager — Backend FastAPI (avec station topographique Topcon)
=============================================================================

  Le robot se deplace en boustrophedon sur une grille X * Y.
  A chaque point il abaisse le prisme AX12, lance une mesure
  sur la station Topcon MS1AX (via XBee), sauvegarde les donnees
  brutes (Hz, V, distance) en base SQLite, puis remonte le prisme
  avant de passer au suivant. Le Z est calcule plus tard (trigo).

  Architecture :
    Tablette <──WebSocket/REST──> Raspberry Pi (ce serveur)
                                       │
                                  USART2 (115200)
                                  STM32 F446RE
                                  (protocole binaire)

  Parcours boustrophedon :
    Rangee 0 (y=0)    : x = 0 → x_max      (gauche a droite)
    Rangee 1 (y=step) : x = x_max → 0       (droite a gauche)
    Rangee 2 (y=2*step): x = 0 → x_max      (gauche a droite)
    ...

  Arret d'urgence :
    - Bouton physique PB14 sur STM32 → ROBOT_ESTOP + ERR_ARU
    - Bouton frontend → POST /stop → CMD_STOP + abort mission
=============================================================================
"""

import asyncio
import os
import sqlite3
import struct
import threading
import serial
import time
from datetime import datetime

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pydantic import BaseModel

app = FastAPI(title="TopoBot Manager API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# =============================================================================
#  CONFIGURATION
# =============================================================================

STM32_PORT = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0671FF554975495067025353-if02"
STM32_BAUD = 115200
# XBee relie en USB (adaptateur FTDI), plus sur les broches TX/RX (UART GPIO).
# /dev/ttyUSB0 = premier adaptateur USB-serie. Verifier le nom exact avec :
#   ls /dev/serial/by-id/      (chemin stable, recommande si plusieurs USB)
#   ou : dmesg | grep ttyUSB
TOPCON_PORT = "/dev/ttyUSB0"
TOPCON_BAUD = 9600

# =============================================================================
#  PROTOCOLE STM32 — identique a protocol.h
# =============================================================================

PROTO_START = 0xAA

CMD_MOVE_TO       = 0x01
CMD_STOP          = 0x02
CMD_RESET_ODOM    = 0x03
CMD_GET_STATUS    = 0x04
CMD_SET_GRID      = 0x05
CMD_DEPLOY_PRISM  = 0x10
CMD_RETRACT_PRISM = 0x11

RESP_ACK    = 0xA1
RESP_DONE   = 0xA2
RESP_STATUS = 0xA3
RESP_ERROR  = 0xA4

STATUS_FMT  = "<fffiiiiffffffB"
STATUS_SIZE = struct.calcsize(STATUS_FMT)

STATE_NAMES = {0: "IDLE", 1: "MOVING", 2: "SETTLING", 3: "DONE", 4: "ESTOP", 5: "ERROR"}
ERR_NAMES = {
    0x01: "ERR_CRC", 0x02: "ERR_UNKNOWN_CMD", 0x03: "ERR_BUSY",
    0x04: "ERR_OUT_BOUNDS", 0x05: "ERR_ARU", 0x06: "ERR_AX12",
}


# =============================================================================
#  ETAT GLOBAL
# =============================================================================

class MissionConfig(BaseModel):
    largeur_x: float
    longueur_y: float
    pas_mesure: float

current_mission = {"x": 10.0, "y": 10.0, "step": 1.0}

abort_flag = threading.Event()
stm32_lock = threading.Lock()


# =============================================================================
#  BASE DE DONNEES SQLITE
# =============================================================================

DB_PATH = os.path.join(os.path.dirname(__file__), "topobot.db")


def init_db():
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    # Pas de Z : la station ne fournit que Hz, V et la distance.
    # Le Z (altitude) est calcule plus tard par formules trigonometriques.
    c.execute("""CREATE TABLE IF NOT EXISTS points
                 (id INTEGER PRIMARY KEY AUTOINCREMENT,
                  time TEXT, x REAL, y REAL,
                  hz REAL, v REAL, dist REAL)""")
    for col in ["hz REAL", "v REAL", "dist REAL"]:
        try:
            c.execute(f"ALTER TABLE points ADD COLUMN {col}")
        except sqlite3.OperationalError:
            pass
    conn.commit()
    conn.close()


init_db()


# =============================================================================
#  STATION TOPCON MS1AX — communication serie via XBee
# =============================================================================

def _read_topcon_line(ser: serial.Serial, timeout: float = 20.0) -> str:
    t_start = time.time()
    t_end = t_start + timeout
    bytes_total = 0
    print(f"[Topcon] Attente reponse (timeout={timeout}s, in_waiting={ser.in_waiting})...")

    while time.time() < t_end:
        if ser.in_waiting > 0:
            raw = ser.readline()
            bytes_total += len(raw)
            elapsed = time.time() - t_start
            hex_dump = " ".join(f"{b:02X}" for b in raw[:50])
            line = raw.decode("utf-8", errors="ignore").strip()

            print(f"[Topcon] [{elapsed:.1f}s] Recu {len(raw)} octets : {repr(line)}")
            print(f"[Topcon]   HEX: {hex_dump}")

            if not line:
                continue

            if "E" in line and len(line) < 5:
                print(f"[Topcon]   -> Erreur moteur/recherche, on ignore")
                time.sleep(0.5)
                continue

            if "," in line:
                print(f"[Topcon]   -> Trame CSV valide detectee apres {elapsed:.1f}s")
                return line

            print(f"[Topcon]   -> Pas de virgule, on continue d'ecouter...")
        else:
            time.sleep(0.1)

    elapsed = time.time() - t_start
    if bytes_total == 0:
        print(f"[Topcon] TIMEOUT ({elapsed:.1f}s) — Aucun octet recu. La station ne repond pas.")
    else:
        print(f"[Topcon] TIMEOUT ({elapsed:.1f}s) — {bytes_total} octets recus mais pas de trame CSV")
    return ""


def _topcon_init_tracking():
    """Reveil de la station + activation du mode tracking (poursuite).
    La station suit le prisme en continu pendant les deplacements du robot.
    """
    ser = get_topcon()
    with topcon_lock:
        print("[Topcon] === INIT TRACKING ===")
        ser.write(b"\r")
        time.sleep(0.2)
        ser.write(b"*PON\r\n")
        time.sleep(2)

        ser.write(b"*RM1\r\n")
        time.sleep(0.5)

        ser.write(b"*/PA 1,1,,\r\n")
        time.sleep(0.5)
        ser.write(b"*/PH 1\r\n")
        time.sleep(0.5)
        ser.write(b"Xe\r\n")
        time.sleep(0.5)

        ser.reset_input_buffer()
        ser.write(b"*ST2\r\n")
        ser.flush()
        time.sleep(0.5)
        ser.reset_input_buffer()
        print("[Topcon] === TRACKING ACTIF ===")


def _topcon_stop_tracking():
    """Arrete le flux de tracking (*ST0) avant une mesure precise."""
    ser = get_topcon()
    with topcon_lock:
        ser.write(b"*ST0\r\n")
        ser.flush()
        time.sleep(0.5)
        ser.reset_input_buffer()
        print("[Topcon] Tracking arrete")


def _topcon_precise_measure() -> dict:
    """Bascule en mode precis, effectue la mesure, retourne {hz, v, dist}.
    Sequence : mode standard -> laser ON -> *ST2 -> parse -> laser OFF.
    """
    ser = get_topcon()
    result = {"hz": 0.0, "v": 0.0, "dist": 0.0}

    with topcon_lock:
        print("[Topcon] === MESURE PRECISE ===")
        ser.write(b"*/PA 1,0,,\r\n")
        time.sleep(0.5)
        ser.write(b"*/PH 0\r\n")
        time.sleep(0.5)
        ser.write(b"Xa\r\n")
        time.sleep(0.5)

        ser.write(b"*GLON\r\n")
        ser.flush()
        time.sleep(2)

        ser.reset_input_buffer()
        ser.write(b"*ST2\r\n")
        ser.flush()

        response = _read_topcon_line(ser, timeout=20)

        ser.write(b"*GLOFF\r\n")
        ser.flush()
        time.sleep(0.5)

    if response and "," in response:
        parts = response.split(",")
        print(f"[Topcon] Parsing : {len(parts)} champs -> {parts}")
        if len(parts) >= 5:
            try:
                result["hz"] = round(float(parts[2]), 4)
                result["v"] = round(float(parts[3]), 4)
                result["dist"] = round(float(parts[4]), 3)
            except ValueError:
                print(f"[Topcon] Erreur parsing : {parts[2:5]}")
    else:
        print("[Topcon] Pas de reponse a la mesure")

    print(f"[Topcon] Hz={result['hz']} V={result['v']} Dist={result['dist']} m")
    return result


def _topcon_resume_tracking():
    """Rebascule en mode tracking apres la mesure precise."""
    ser = get_topcon()
    with topcon_lock:
        print("[Topcon] === REPRISE TRACKING ===")
        ser.write(b"*/PA 1,1,,\r\n")
        time.sleep(0.5)
        ser.write(b"*/PH 1\r\n")
        time.sleep(0.5)
        ser.write(b"Xe\r\n")
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.write(b"*ST2\r\n")
        ser.flush()
        time.sleep(0.5)
        ser.reset_input_buffer()
        print("[Topcon] Tracking repris")


def _topcon_measure() -> float:
    """Mesure simple (mode manuel) — ouvre/ferme le port a chaque appel."""
    print(f"[Topcon] === DEBUT MESURE ===")
    try:
        ser = serial.Serial(TOPCON_PORT, TOPCON_BAUD, timeout=2)
        time.sleep(1)
        ser.reset_input_buffer()

        ser.write(b"*GLON\r\n")
        ser.flush()
        time.sleep(1.5)

        ser.reset_input_buffer()
        ser.write(b"*ST2\r\n")
        ser.flush()

        response = _read_topcon_line(ser, timeout=20)

        ser.write(b"*GLOFF\r\n")
        ser.flush()
        time.sleep(0.5)
        ser.close()

        if not response:
            return 0.0

        parts = response.split(",")
        if len(parts) >= 5:
            try:
                return round(float(parts[4]), 3)
            except ValueError:
                pass
        return 0.0

    except Exception as e:
        print(f"[Topcon] EXCEPTION : {e}")
        return 0.0


def _topcon_standard_mode():
    """Configuration mode standard (manuel)."""
    print(f"[Topcon] === MODE STANDARD ===")
    try:
        ser = serial.Serial(TOPCON_PORT, TOPCON_BAUD, timeout=2)
        time.sleep(2)

        for cmd, label in [(b"*RM1\r\n", "RM1"), (b"*/PA 1,0,,\r\n", "PA"),
                           (b"*/PH 0\r\n", "PH"), (b"Xa\r\n", "Xa")]:
            ser.write(cmd)
            ser.flush()
            print(f"[Topcon] Envoi {label}")
            time.sleep(1)
            if ser.in_waiting > 0:
                ser.read(ser.in_waiting)

        ser.close()
        print(f"[Topcon] === MODE STANDARD OK ===")
    except Exception as e:
        print(f"[Topcon] EXCEPTION mode standard : {e}")
    return True


# =============================================================================
#  PROTOCOLE — encodage / decodage des trames binaires STM32
# =============================================================================

def _crc(cmd: int, payload: bytes) -> int:
    c = cmd ^ len(payload)
    for b in payload:
        c ^= b
    return c & 0xFF


def _build_frame(cmd: int, payload: bytes = b"") -> bytes:
    return bytes([PROTO_START, cmd, len(payload)]) + payload + bytes([_crc(cmd, payload)])


def _read_frame(ser: serial.Serial, timeout: float = 1.0):
    """Lit une trame complete depuis le port serie.
    Retourne (resp_code, data_bytes) ou None si timeout.
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
        resp, length = hdr[0], hdr[1]
        if length > 64:
            continue
        rest = ser.read(length + 1)
        if len(rest) != length + 1:
            continue
        data = rest[:length]
        if rest[length] != _crc(resp, data):
            continue
        return resp, data
    return None


def _stm32_send_cmd(ser: serial.Serial, cmd: int, label: str = "", timeout: float = 2.0) -> bool:
    """Envoie une commande sans payload et attend RESP_ACK."""
    frame = _build_frame(cmd)
    with stm32_lock:
        ser.reset_input_buffer()
        ser.write(frame)
        ser.flush()
        r = _read_frame(ser, timeout)
    if r is None:
        print(f"[STM32] {label} : pas de reponse")
        return False
    code, data = r
    if code == RESP_ACK:
        print(f"[STM32] {label} : ACK")
        return True
    if code == RESP_ERROR:
        print(f"[STM32] {label} : {ERR_NAMES.get(data[0] if data else 0, '?')}")
        return False
    return False


def _stm32_move_to(ser: serial.Serial, x_m: float, y_m: float,
                   theta: float = 0.0, timeout: float = 60.0) -> bool:
    """Envoie CMD_MOVE_TO et attend RESP_DONE ou STATUS state=3.
    Verifie abort_flag a chaque iteration pour permettre l'arret immediat.
    """
    payload = struct.pack("<fff", x_m, y_m, theta)
    frame = _build_frame(CMD_MOVE_TO, payload)

    with stm32_lock:
        ser.reset_input_buffer()
        ser.write(frame)
        ser.flush()
        print(f"[STM32] MOVE_TO x={x_m:.3f} y={y_m:.3f}")

        r = _read_frame(ser, timeout=2.0)
        if r is None:
            print("[STM32] MOVE_TO : pas de reponse")
            return False
        code, data = r
        if code == RESP_ERROR:
            print(f"[STM32] MOVE_TO refuse : {ERR_NAMES.get(data[0] if data else 0, '?')}")
            return False
        if code != RESP_ACK:
            return False

        print("[STM32] MOVE_TO ACK — attente DONE...")

        deadline = time.time() + timeout
        while time.time() < deadline:
            if abort_flag.is_set():
                print("[STM32] MOVE_TO interrompu par abort")
                return False

            r = _read_frame(ser, timeout=1.0)
            if r is None:
                continue
            code, data = r

            if code == RESP_DONE:
                print("[STM32] DONE recu")
                return True

            if code == RESP_STATUS and len(data) >= STATUS_SIZE:
                state = struct.unpack_from(STATUS_FMT, data, 0)[13]
                if state == 3:
                    return True
                if state in (4, 5):
                    print(f"[STM32] ESTOP/ERROR (state={state})")
                    return False

            if code == RESP_ERROR:
                print(f"[STM32] ERREUR : {ERR_NAMES.get(data[0] if data else 0, '?')}")
                return False

    print("[STM32] Timeout MOVE_TO")
    return False


def _stm32_get_status(ser: serial.Serial) -> dict | None:
    """Interroge la position et l'etat du robot."""
    frame = _build_frame(CMD_GET_STATUS)
    with stm32_lock:
        ser.reset_input_buffer()
        ser.write(frame)
        ser.flush()
        r = _read_frame(ser, timeout=1.0)
    if r is None:
        return None
    code, data = r
    if code != RESP_STATUS or len(data) < STATUS_SIZE:
        return None
    f = struct.unpack_from(STATUS_FMT, data, 0)
    return {
        "x": f[7], "y": f[8], "theta": f[9],
        "state": f[13], "state_name": STATE_NAMES.get(f[13], "?"),
    }


# =============================================================================
#  WAYPOINTS BOUSTROPHEDON
# =============================================================================

def generate_waypoints(x_max: float, y_max: float, step: float) -> list[tuple[float, float]]:
    """Genere les waypoints en serpentin.
    Inclut le point (0,0) en premier — le robot y est deja au depart.
    """
    if step <= 0:
        step = 1.0
    waypoints = []
    y = 0.0
    row = 0
    while y <= y_max + 1e-9:
        xs = []
        x = 0.0
        while x <= x_max + 1e-9:
            xs.append(round(x, 3))
            x += step
        if row % 2 == 1:
            xs.reverse()
        for xi in xs:
            waypoints.append((xi, round(y, 3)))
        y += step
        row += 1
    return waypoints


# =============================================================================
#  CONNEXION STM32 PERSISTANTE
# =============================================================================

stm32_ser: serial.Serial | None = None

def get_stm32() -> serial.Serial:
    global stm32_ser
    if stm32_ser is None or not stm32_ser.is_open:
        stm32_ser = serial.Serial(STM32_PORT, STM32_BAUD, timeout=1)
        time.sleep(0.3)
        stm32_ser.reset_input_buffer()
        print(f"[STM32] Port ouvert : {STM32_PORT}")
    return stm32_ser


# =============================================================================
#  CONNEXION TOPCON PERSISTANTE
# =============================================================================

topcon_ser: serial.Serial | None = None
topcon_lock = threading.Lock()


def get_topcon() -> serial.Serial:
    global topcon_ser
    if topcon_ser is None or not topcon_ser.is_open:
        topcon_ser = serial.Serial(TOPCON_PORT, TOPCON_BAUD, timeout=2)
        time.sleep(1)
        topcon_ser.reset_input_buffer()
        print(f"[Topcon] Port ouvert : {TOPCON_PORT}")
    return topcon_ser


# =============================================================================
#  ROUTES REST
# =============================================================================

@app.post("/start-mission")
def start_mission(config: MissionConfig):
    global current_mission
    current_mission = {
        "x": config.largeur_x,
        "y": config.longueur_y,
        "step": config.pas_mesure,
    }
    abort_flag.clear()

    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute("DELETE FROM points")
    conn.commit()
    conn.close()

    print(f"[Mission] Config : {config.largeur_x}x{config.longueur_y}m pas={config.pas_mesure}m")
    return {"status": "success", "message": "Mission configuree"}


@app.post("/stop")
def stop_robot():
    """Arret d'urgence : coupe la mission + envoie CMD_STOP au STM32.
    Bypass du lock serie : on ecrit CMD_STOP directement pour ne pas attendre
    la fin d'un MOVE_TO en cours (qui detient stm32_lock jusqu'a 90 s).
    L'ecriture concurrente est sure : le thread MOVE_TO ne fait que lire
    pendant le mouvement (il n'ecrit qu'au tout debut).
    """
    abort_flag.set()
    ok = False
    try:
        ser = get_stm32()
        frame = _build_frame(CMD_STOP)
        ser.write(frame)
        ser.flush()
        print("[STM32] STOP d'urgence envoye (bypass lock)")
        ok = True
    except Exception as e:
        print(f"[STM32] Erreur STOP urgence : {e}")
    try:
        tser = get_topcon()
        tser.write(b"*ST0\r\n")
        tser.flush()
        print("[Topcon] Tracking coupe par arret d'urgence")
    except Exception:
        pass
    return {
        "status": "success" if ok else "error",
        "message": "Arret d'urgence envoye" if ok else "Erreur STM32",
    }


@app.get("/status")
def get_status():
    try:
        ser = get_stm32()
        s = _stm32_get_status(ser)
    except Exception:
        s = None
    if s:
        return {"status": "success", "robot": s}
    return {"status": "error", "message": "STM32 ne repond pas"}


@app.post("/topcon/std")
def topcon_std():
    _topcon_standard_mode()
    return {"status": "success", "message": "Mode standard active"}


@app.post("/topcon/mes")
def topcon_mes():
    dist = _topcon_measure()
    return {"status": "success", "distance": dist}


@app.post("/topcon/track")
def topcon_track():
    """Active le mode tracking manuellement."""
    try:
        _topcon_init_tracking()
        return {"status": "success", "message": "Mode tracking active"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@app.post("/topcon/track/stop")
def topcon_track_stop():
    """Arrete le mode tracking manuellement."""
    try:
        _topcon_stop_tracking()
        return {"status": "success", "message": "Tracking arrete"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@app.get("/results")
def get_results():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    c = conn.cursor()
    c.execute("SELECT * FROM points")
    rows = c.fetchall()
    conn.close()
    points = []
    for index, row in enumerate(rows):
        points.append({
            "id": f"PT_{index}",
            "time": row["time"],
            "x": row["x"],
            "y": row["y"],
            "hz": row["hz"],
            "v": row["v"],
            "dist": row["dist"],
        })
    return {"points": points}


# =============================================================================
#  WEBSOCKET MISSION
#
#  Pour chaque point de la grille :
#    1. MOVE_TO (sauf pour le premier point (0,0) ou le robot est deja)
#    2. DEPLOY_PRISM  (abaisser le prisme AX12)
#    3. Mesure Topcon MS1AX → Hz, V, distance
#    4. RETRACT_PRISM (remonter le prisme)
#    5. Sauvegarde (time, X, Y, Hz, V, dist) en SQLite
#    6. Passer au point suivant
# =============================================================================

@app.websocket("/ws")
async def websocket_mission(websocket: WebSocket):
    await websocket.accept()
    loop = asyncio.get_event_loop()

    x_max = float(current_mission["x"])
    y_max = float(current_mission["y"])
    step = float(current_mission["step"])

    waypoints = generate_waypoints(x_max, y_max, step)
    total = len(waypoints)

    async def log(msg: str):
        print(f"[Mission] {msg}")
        try:
            await websocket.send_json({"type": "log", "message": msg})
        except Exception:
            pass

    async def check_abort_ws():
        """Verifie si le frontend a envoye un abort via WebSocket."""
        try:
            data = await asyncio.wait_for(websocket.receive_json(), timeout=0.05)
            if data.get("action") == "abort":
                abort_flag.set()
        except (asyncio.TimeoutError, WebSocketDisconnect, Exception):
            pass

    try:
        ser = get_stm32()

        await log("Arret de securite...")
        await loop.run_in_executor(None, _stm32_send_cmd, ser, CMD_STOP, "STOP", 2.0)

        await log("Reset odometrie...")
        await loop.run_in_executor(None, _stm32_send_cmd, ser, CMD_RESET_ODOM, "RESET_ODOM", 2.0)
        await asyncio.sleep(1.0)

        # -- Initialisation station Topcon en mode tracking --
        await log("Initialisation station Topcon en mode tracking...")
        await websocket.send_json({"type": "tracking", "active": True})
        try:
            await loop.run_in_executor(None, _topcon_init_tracking)
            await log("Station en mode tracking — suivi du prisme actif")
        except Exception as e:
            await log(f"ERREUR init tracking : {e}")
            await websocket.send_json({"type": "error", "message": f"Init tracking echoue: {e}"})
            return

        abort_flag.clear()
        await log(f"Mission lancee : {total} points")

        for idx, (wp_x, wp_y) in enumerate(waypoints):
            await check_abort_ws()
            if abort_flag.is_set():
                await log("MISSION INTERROMPUE")
                await loop.run_in_executor(None, _topcon_stop_tracking)
                await loop.run_in_executor(None, _stm32_send_cmd, ser, CMD_STOP, "STOP", 2.0)
                await websocket.send_json({"type": "tracking", "active": False})
                await websocket.send_json({"type": "aborted"})
                return

            pts_done = idx + 1
            progress = int((pts_done / total) * 100)

            # -- Deplacement (sauf premier point : le robot y est deja) --
            # La station suit le prisme en mode tracking pendant le deplacement
            if idx > 0:
                await log(f"[{pts_done}/{total}] Deplacement vers X={wp_x:.3f} Y={wp_y:.3f}")
                ok = await loop.run_in_executor(
                    None, _stm32_move_to, ser, wp_x, wp_y, 0.0, 90.0
                )
                if abort_flag.is_set():
                    await log("MISSION INTERROMPUE")
                    await loop.run_in_executor(None, _topcon_stop_tracking)
                    await loop.run_in_executor(None, _stm32_send_cmd, ser, CMD_STOP, "STOP", 2.0)
                    await websocket.send_json({"type": "tracking", "active": False})
                    await websocket.send_json({"type": "aborted"})
                    return
                if not ok:
                    await log(f"ERREUR deplacement vers X={wp_x} Y={wp_y}")
                    await loop.run_in_executor(None, _topcon_stop_tracking)
                    await websocket.send_json({"type": "tracking", "active": False})
                    await websocket.send_json({"type": "error", "message": "Deplacement echoue"})
                    return
            else:
                await log(f"[{pts_done}/{total}] Point de depart (0, 0) — robot deja en place")

            # -- Arret tracking pour mesure precise --
            await log("Arret tracking pour mesure precise...")
            await loop.run_in_executor(None, _topcon_stop_tracking)
            await websocket.send_json({"type": "tracking", "active": False})

            # -- Abaissement prisme AX12 --
            await log(f"Deploiement prisme au point X={wp_x} Y={wp_y}...")
            await loop.run_in_executor(
                None, _stm32_send_cmd, ser, CMD_DEPLOY_PRISM, "DEPLOY_PRISM", 5.0
            )
            await asyncio.sleep(2.5)

            # -- Verification abort avant mesure --
            await check_abort_ws()
            if abort_flag.is_set():
                await log("MISSION INTERROMPUE")
                await loop.run_in_executor(None, _stm32_send_cmd, ser, CMD_RETRACT_PRISM, "RETRACT", 5.0)
                await loop.run_in_executor(None, _stm32_send_cmd, ser, CMD_STOP, "STOP", 2.0)
                await websocket.send_json({"type": "aborted"})
                return

            # -- Mesure precise Topcon --
            await log("Mesure precise station topographique...")
            measure = await loop.run_in_executor(None, _topcon_precise_measure)
            heure = datetime.now().strftime("%H:%M:%S")
            await log(f"Mesure : Hz={measure['hz']} gon | V={measure['v']} gon | Dist={measure['dist']} m")

            # -- Remontee prisme --
            await log("Repli du prisme...")
            await loop.run_in_executor(
                None, _stm32_send_cmd, ser, CMD_RETRACT_PRISM, "RETRACT_PRISM", 5.0
            )
            await asyncio.sleep(1.5)

            # -- Sauvegarde en base de donnees --
            conn = sqlite3.connect(DB_PATH)
            c = conn.cursor()
            c.execute(
                "INSERT INTO points (time, x, y, hz, v, dist) VALUES (?, ?, ?, ?, ?, ?)",
                (heure, wp_x, wp_y, measure["hz"], measure["v"], measure["dist"]),
            )
            conn.commit()
            conn.close()

            # -- Reprise tracking pour le prochain deplacement --
            if idx < total - 1:
                await log("Reprise tracking...")
                await loop.run_in_executor(None, _topcon_resume_tracking)
                await websocket.send_json({"type": "tracking", "active": True})

            # -- Notification frontend --
            await websocket.send_json({
                "type": "position",
                "x": wp_x,
                "y": wp_y,
                "hz": measure["hz"],
                "v": measure["v"],
                "dist": measure["dist"],
                "time": heure,
                "progress": progress,
            })
            await log(f"Point {pts_done}/{total} termine — Dist={measure['dist']:.3f} m")

        # -- Arret tracking final --
        await log("Arret tracking final...")
        try:
            await loop.run_in_executor(None, _topcon_stop_tracking)
        except Exception:
            pass
        await websocket.send_json({"type": "tracking", "active": False})

        # -- Retour origine --
        await log("Retour a l'origine (0, 0)...")
        await loop.run_in_executor(None, _stm32_move_to, ser, 0.0, 0.0, 0.0, 90.0)

        await log("Mission terminee")
        await websocket.send_json({"type": "done"})

    except WebSocketDisconnect:
        print("[WS] Deconnexion")
        try:
            _stm32_send_cmd(get_stm32(), CMD_STOP, "STOP (deconnexion)")
        except Exception:
            pass
    except Exception as e:
        print(f"[WS] Erreur : {e}")
        try:
            await websocket.send_json({"type": "error", "message": str(e)})
        except Exception:
            pass


# =============================================================================
#  FRONTEND STATIQUE
# =============================================================================

chemin_frontend = os.path.join(os.path.dirname(__file__), "../frontend/dist")

if os.path.isdir(chemin_frontend):
    app.mount("/assets", StaticFiles(directory=os.path.join(chemin_frontend, "assets")), name="assets")

    @app.get("/{catchall:path}")
    def serve_react_app(catchall: str):
        return FileResponse(os.path.join(chemin_frontend, "index.html"))
else:
    print("[WARN] frontend/dist inexistant — lancer npm run build")
