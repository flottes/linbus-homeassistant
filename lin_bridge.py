#!/usr/bin/env python3
"""
lin_bridge.py — Python-Client für die ESP32 LIN-Bridge (Variante C).

Läuft auf dem Raspberry Pi. Der Pi ist der Scheduler: er entscheidet,
welche ID wann abgefragt wird, und schickt der ESP32-Bridge einzelne
READ-Kommandos. Der ESP32 macht nur das zeitkritische LIN-Timing.

Modi:
    scan                 einmal alle IDs 0x00..0x3F auflisten
    read <id>            eine ID einmal abfragen
    monitor              Multi-ID-Scheduler, Ausgabe auf Konsole
    mqtt                 Betriebsmodus: pollt alle IDs, published nach
                         Home Assistant (Auto-Discovery). Config oben im Script.
    inspect <id>         Live-Dissector für EINE ID: Bit-Matrix, Endian-Deutungen,
                         Change-Tracking; Bildschirm aktualisiert sich in-place

Beispiele:
    python3 lin_bridge.py --port /dev/lin-bridge scan
    python3 lin_bridge.py --port /dev/lin-bridge read 0C
    python3 lin_bridge.py --port /dev/lin-bridge monitor
    python3 lin_bridge.py --port /dev/lin-bridge mqtt
    python3 lin_bridge.py --port /dev/lin-bridge inspect 0C

Abhängigkeiten:  pip install pyserial paho-mqtt
"""

import argparse
import json
import time

import serial  # pyserial


# ====================================================================
#  Monitor-Plan: welche IDs im Betrieb, mit Intervall
# ====================================================================
POLL_PLAN = [
    # (id,   intervall_s,  name)
    (0x16,   1.0,          "votronic_current"),
    (0x20,   1.0,          "votronic_solar"),
    (0x0C,   10.0,         "dometic_fridge"),
    # (0x19, 5.0,          "votronic_unknown19"),
]


# ====================================================================
#  MQTT / Home-Assistant-Konfiguration
#  --> hier die Zugangsdaten und Broker-Adresse eintragen
# ====================================================================
MQTT_HOST     = "192.168.15.1"
MQTT_PORT     = 1883
MQTT_USER     = "DEIN_MQTT_USER"      # <-- anpassen
MQTT_PASS     = "DEIN_MQTT_PASS"      # <-- anpassen
MQTT_PREFIX   = "van/lin"             # Basis-Topic für die Messwerte
DISCOVERY_PREFIX = "homeassistant"    # HA-Standard für Auto-Discovery

# Availability-Glättung: eine Quelle gilt erst als "nicht verfügbar", wenn
# so viele Polls IN FOLGE fehlschlagen. Das entkoppelt die Bewertung vom
# Poll-Takt und verhindert Flackern durch einzelne verschluckte Frames.
#   Dometic @10s * 6 = 60s Toleranz, bevor "unavailable"
#   Votronic @1s * 60 = 60s Toleranz (schläft ohne Ladequelle)
AVAILABILITY_MAX_FAILS = {
    "dometic_fridge":   6,
    "votronic_current": 60,
    "votronic_solar":   60,
}
AVAILABILITY_MAX_FAILS_DEFAULT = 10

# ---- Sensor-Definitionen für Home Assistant Auto-Discovery ----------
# Pro decodiertem Schlüssel: wie er in HA erscheinen soll.
#   kind:   "sensor" oder "binary_sensor"
#   name:   Anzeigename in HA
#   unit:   Einheit (nur sensor)
#   dclass: device_class (optional, für Icon/Verhalten)
#   source: welche POLL_PLAN-Quelle liefert den Wert (für Availability)
SENSOR_DEFS = {
    # --- Votronic Strom (0x16) ---
    "current_a":     dict(kind="sensor", name="Votronic Strom",
                          unit="A", dclass="current", source="votronic_current"),
    "charging":      dict(kind="binary_sensor", name="Votronic lädt",
                          dclass="battery_charging", source="votronic_current"),
    # --- Votronic Solar (0x20) ---
    "solar_w":       dict(kind="sensor", name="Votronic Solarleistung",
                          unit="W", dclass="power", source="votronic_solar"),
    "status":        dict(kind="sensor", name="Votronic Laderegler-Status",
                          unit=None, dclass=None, source="votronic_solar"),
    # --- Dometic Kühlschrank (0x0C) ---
    "cooling_level": dict(kind="sensor", name="Kühlschrank Stufe",
                          unit=None, dclass=None, source="dometic_fridge"),
    "mode":          dict(kind="sensor", name="Kühlschrank Modus",
                          unit=None, dclass=None, source="dometic_fridge"),
    "power_on":      dict(kind="binary_sensor", name="Kühlschrank ein",
                          dclass="power", source="dometic_fridge"),
    "compressor":    dict(kind="binary_sensor", name="Kühlschrank Kompressor",
                          dclass="running", source="dometic_fridge"),
    "door_open":     dict(kind="binary_sensor", name="Kühlschrank Tür",
                          dclass="door", source="dometic_fridge"),
}

# Werte, die zwar decodiert werden, aber NICHT nach HA sollen (Rohdaten etc.)
SKIP_KEYS = {"raw", "status_raw"}


# ====================================================================
#  ANSI-Helfer (für den in-place Live-Screen)
# ====================================================================
class A:
    CLEAR   = "\033[2J"
    HOME    = "\033[H"
    HIDE    = "\033[?25l"
    SHOW    = "\033[?25h"
    RESET   = "\033[0m"
    BOLD    = "\033[1m"
    DIM     = "\033[2m"
    RED     = "\033[31m"
    GREEN   = "\033[32m"
    YELLOW  = "\033[33m"
    BLUE    = "\033[34m"
    CYAN    = "\033[36m"
    INVERT  = "\033[7m"

    @staticmethod
    def clearline():
        return "\033[K"


# ====================================================================
#  Serielle Bridge-Anbindung
# ====================================================================
class LinBridge:
    def __init__(self, port, baud=115200, timeout=1.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2.0)                 # ESP32-Reset abwarten
        self.ser.reset_input_buffer()

    def send(self, cmd):
        try:
            self.ser.write((cmd.strip() + "\n").encode())
            self.ser.flush()
        except serial.SerialException:
            pass

    def readline(self):
        """Liest eine Zeile. Fängt den 'device reports readiness ...'-Fehler
        ab, der auf manchen Pi/ACM-Treibern bei Timeout auftritt, und gibt
        in dem Fall '' zurück statt zu crashen."""
        try:
            raw = self.ser.readline()
        except serial.SerialException:
            return ""
        if not raw:
            return ""
        return raw.decode(errors="replace").strip()

    def read_id(self, id_byte, tries=3):
        """Fragt eine ID einmal ab. Robust gegen Timeouts und träge Slaves:
        pro Versuch wird der Puffer geleert, gesendet, kurz gewartet und dann
        auf die RESP/EMPTY-Zeile gelauscht."""
        for _ in range(tries):
            try:
                self.ser.reset_input_buffer()
            except serial.SerialException:
                pass
            self.send(f"READ {id_byte:02X}")
            deadline = time.time() + 0.6
            empty_seen = False
            while time.time() < deadline:
                line = self.readline()
                if not line:
                    continue
                if line.startswith("RESP"):
                    f = parse_resp(line)
                    if f is not None:
                        return f
                elif line.startswith("EMPTY"):
                    empty_seen = True
                    break
                # OK/ERR/andere Zeilen ignorieren und weiterlauschen
            # nur kurz warten und erneut versuchen
            time.sleep(0.03)
        return None

    def close(self):
        self.ser.close()


def parse_resp(line):
    """RESP <id> <pid> <len> <b0..bn> <crc> <cs>  ->  Dict oder None."""
    parts = line.split()
    if not parts or parts[0] != "RESP":
        return None
    try:
        rid = int(parts[1], 16)
        pid = int(parts[2], 16)
        length = int(parts[3])
        data = [int(x, 16) for x in parts[4:4 + length]]
        crc = int(parts[4 + length], 16)
        cs = parts[5 + length] if len(parts) > 5 + length else "?"
        return {"id": rid, "pid": pid, "data": data, "crc": crc, "cs": cs}
    except (ValueError, IndexError):
        return None


# ====================================================================
#  Decoder – hier wächst dein Wissen über die Geräte
# ====================================================================
def decode(frame):
    if frame is None:
        return {}
    rid, d = frame["id"], frame["data"]
    out = {}

    if rid == 0x16 and len(d) >= 2:
        out["current_a"] = (d[0] | (d[1] << 8)) / 10.0
        if len(d) >= 3:
            out["charging"] = bool(d[2] & 0x10)

    elif rid == 0x20 and len(d) >= 5:
        raw = d[0] | (d[1] << 8) | (d[2] << 16) | (d[3] << 24)
        out["solar_w"] = raw / 47500.0
        st = d[4]
        out["status_raw"] = st
        out["status"] = {
            0x40: "offline/sleep", 0x44: "bulk",
            0x4D: "bulk+absorb",   0x5D: "absorb",
        }.get(st, f"0x{st:02X}")

    elif rid == 0x0C and len(d) >= 8:
        # Dometic RC 10.4T 70 (Kompressor, CI-Bus) — verifiziert:
        out["cooling_level"] = d[1]                 # B1: Stufe 1..5 direkt
        out["power_on"]      = bool(d[0] & 0x40)    # B0 Bit6: Gerät ein
        out["compressor"]    = bool(d[7] & 0x80)    # B7 Bit7: an/Kompressor
        out["door_open"]     = bool(d[2] & 0x20)    # B2 Bit5: Tür offen
        # B0 Bits 3-4: Betriebsmodus (2-Bit-Feld)
        mode_bits = (d[0] >> 3) & 0x03
        out["mode"] = {
            0b00: "performance",
            0b10: "silent",
            0b11: "boost",
        }.get(mode_bits, f"unknown({mode_bits:02b})")
        # noch unklar: B2 Bit1 (0x02) wechselt gelegentlich — evtl. Kühlanforderung.
        # Ist-Temperatur bisher nicht gefunden (B4/B5 stehen auf 0).
        out["raw"] = " ".join(f"{b:02X}" for b in d)

    else:
        out["raw"] = " ".join(f"{b:02X}" for b in d)

    return out


# ====================================================================
#  Multi-ID-Scheduler (Betriebsmodus)
# ====================================================================
class MultiPoller:
    def __init__(self, bridge, plan, on_result):
        self.bridge = bridge
        self.on_result = on_result
        self.plan = [{"id": i, "interval": iv, "name": nm, "next": 0.0}
                     for (i, iv, nm) in plan]

    def run_forever(self):
        while True:
            now = time.time()
            due = [p for p in self.plan if now >= p["next"]]
            if due:
                due.sort(key=lambda p: p["next"])
                p = due[0]
                frame = self.bridge.read_id(p["id"])
                values = decode(frame) if frame else {}
                self.on_result(p["name"], p["id"], frame, values)
                p["next"] = now + p["interval"]
            else:
                nxt = min(p["next"] for p in self.plan)
                time.sleep(max(0.0, min(0.2, nxt - now)))


def print_result(name, id_byte, frame, values):
    ts = time.strftime("%H:%M:%S")
    if frame is None:
        print(f"{ts}  {name:20s} 0x{id_byte:02X}  (keine Antwort / schläft)")
    else:
        print(f"{ts}  {name:20s} 0x{id_byte:02X}  cs={frame['cs']:7s} {values}")


# ====================================================================
#  LIVE-DISSECTOR  (inspect-Modus)
#  Pollt eine ID dauerhaft und rendert ein festes Dashboard, das sich
#  in-place aktualisiert. Verfolgt, welche Bits/Bytes sich je geändert
#  haben — der schnellste Weg, digitale Zustände (Tür auf/zu) und
#  Messwerte zu finden.
# ====================================================================
class Inspector:
    def __init__(self, bridge, id_byte, interval=0.5):
        self.bridge = bridge
        self.id = id_byte
        self.interval = interval
        # Change-Tracking
        self.first = None          # erstes gesehenes Frame (Referenz)
        self.prev = None           # vorheriges Frame (für "gerade geändert")
        self.ever_changed = None   # pro Byte: Bitmaske der je veränderten Bits
        self.bmin = None           # pro Byte: min
        self.bmax = None           # pro Byte: max
        self.frames = 0
        self.empties = 0
        self.start = time.time()

    def update_stats(self, data):
        n = len(data)
        if self.first is None:
            self.first = list(data)
            self.ever_changed = [0] * n
            self.bmin = list(data)
            self.bmax = list(data)
        # Länge kann theoretisch wechseln -> defensiv angleichen
        if len(self.ever_changed) < n:
            grow = n - len(self.ever_changed)
            self.ever_changed += [0] * grow
            self.bmin += data[len(self.bmin):]
            self.bmax += data[len(self.bmax):]
            self.first += data[len(self.first):]
        for i in range(n):
            self.ever_changed[i] |= (data[i] ^ self.first[i])
            self.bmin[i] = min(self.bmin[i], data[i])
            self.bmax[i] = max(self.bmax[i], data[i])

    def render(self, frame):
        d = frame["data"] if frame else []
        n = len(d)
        prev = self.prev or []
        lines = []

        el = time.time() - self.start
        lines.append(f"{A.BOLD}{A.CYAN}=== LIN Inspector  ID 0x{self.id:02X}  "
                     f"(PID 0x{frame['pid']:02X}){A.RESET}"
                     if frame else
                     f"{A.BOLD}{A.CYAN}=== LIN Inspector  ID 0x{self.id:02X}{A.RESET}")
        status = (f"cs={frame['cs']}" if frame else f"{A.RED}keine Antwort{A.RESET}")
        lines.append(f"frames={self.frames}  empty={self.empties}  "
                     f"laufzeit={el:5.0f}s  {status}  "
                     f"{A.DIM}(Strg-C beendet){A.RESET}")
        lines.append("")

        if not frame:
            lines.append(f"{A.YELLOW}Gerät antwortet gerade nicht "
                         f"({self.empties} leer in Folge).{A.RESET}")
            lines.append(f"{A.DIM}Falls dauerhaft: Baudrate/Verkabelung prüfen, "
                         f"oder Gerät im Standby (Votronic ohne Ladequelle).{A.RESET}")
            self._flush(lines)
            return

        # ---- Rohbytes-Zeile, geänderte Bytes hervorgehoben ----
        hexparts, decparts = [], []
        for i in range(n):
            changed_now = (i < len(prev) and prev[i] != d[i])
            col = (A.INVERT if changed_now else "")
            hexparts.append(f"{col}{d[i]:02X}{A.RESET}")
            decparts.append(f"{col}{d[i]:3d}{A.RESET}")
        lines.append("Byte:   " + "  ".join(f" B{i} " for i in range(n)))
        lines.append("hex :   " + "  ".join(f" {h} " for h in hexparts))
        lines.append("dec :   " + "  ".join(f"{v}" for v in decparts))
        lines.append("")

        # ---- Bit-Matrix ----
        # Spalten = Bit7..Bit0; markiert: 1=grün, 0=dim, "je geändert"=gelb/invert
        lines.append(f"{A.BOLD}Bit-Matrix{A.RESET}   "
                     f"(grün=1, {A.DIM}grau=0{A.RESET}, "
                     f"{A.YELLOW}gelb=Bit hat sich mal geändert{A.RESET})")
        header = "        " + "  ".join(f"b{b}" for b in range(7, -1, -1))
        lines.append(A.DIM + header + A.RESET)
        for i in range(n):
            cells = []
            for b in range(7, -1, -1):
                bitval = (d[i] >> b) & 1
                everch = (self.ever_changed[i] >> b) & 1 if i < len(self.ever_changed) else 0
                if bitval:
                    cell = (A.YELLOW if everch else A.GREEN) + " 1" + A.RESET
                else:
                    cell = (A.YELLOW if everch else A.DIM) + " 0" + A.RESET
                cells.append(cell)
            # welche Bits dieses Bytes sind je gewandert -> als Maske hinten
            mask = self.ever_changed[i] if i < len(self.ever_changed) else 0
            tag = f"  {A.DIM}chg-mask=0x{mask:02X}{A.RESET}" if mask else ""
            lines.append(f"  B{i} 0x{d[i]:02X}  " + "  ".join(cells) + tag)
        lines.append("")

        # ---- Zahlen-Interpretationen ----
        lines.append(f"{A.BOLD}Zahlen-Deutungen{A.RESET}")
        # einzelne Bytes signed
        s8 = "  ".join(f"B{i}={_s8(d[i]):4d}" for i in range(n))
        lines.append(f"  int8 :  {s8}")
        # 16-bit Paare (aufeinanderfolgend, überlappend praktischer: 0/1,1/2,2/3..)
        lines.append(f"  16-bit Paare (LE / BE, unsigned / signed):")
        for i in range(n - 1):
            lo, hi = d[i], d[i + 1]
            le = lo | (hi << 8)
            be = (lo << 8) | hi
            lines.append(
                f"    B{i}/B{i+1}: "
                f"LE={le:5d} ({_s16(le):6d})   "
                f"BE={be:5d} ({_s16(be):6d})   "
                f"{A.DIM}LE/10={le/10:.1f}  LE/100={le/100:.2f}{A.RESET}"
            )
        # 32-bit über erste vier Bytes
        if n >= 4:
            le32 = d[0] | (d[1] << 8) | (d[2] << 16) | (d[3] << 24)
            be32 = (d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3]
            lines.append(f"  32-bit B0..B3:  LE={le32}  BE={be32}"
                         f"   {A.DIM}LE/47500={le32/47500:.3f}{A.RESET}")
        lines.append("")

        # ---- Min/Max je Byte (zeigt, welches Byte 'lebt') ----
        lines.append(f"{A.BOLD}Byte-Aktivität{A.RESET} "
                     f"{A.DIM}(min..max über Laufzeit; span>0 = ändert sich){A.RESET}")
        for i in range(n):
            span = self.bmax[i] - self.bmin[i]
            bar = _bar(d[i], self.bmin[i], self.bmax[i])
            hot = A.GREEN if span > 0 else A.DIM
            lines.append(f"  B{i}: {hot}{self.bmin[i]:3d}..{self.bmax[i]:3d}"
                         f"  span={span:3d}{A.RESET}  {bar}")

        self._flush(lines)

    def _flush(self, lines):
        out = A.HOME
        for ln in lines:
            out += ln + A.clearline() + "\n"
        out += "\033[J"   # Rest des Schirms löschen
        print(out, end="", flush=True)

    def run(self):
        print(A.CLEAR + A.HIDE, end="")
        try:
            while True:
                frame = self.bridge.read_id(self.id)
                if frame:
                    self.frames += 1
                    self.empties = 0            # Zähler bei Erfolg zurücksetzen
                    self.update_stats(frame["data"])
                    self.render(frame)
                    self.prev = list(frame["data"])
                else:
                    self.empties += 1
                    self.render(None)
                time.sleep(self.interval)
        except KeyboardInterrupt:
            pass
        except serial.SerialException as e:
            print(A.SHOW + A.RESET)
            print(f"\n# Serieller Fehler: {e}")
            print("# Tipp: ESP kurz ab-/anstecken, Port prüfen, dann neu starten.")
            return
        finally:
            print(A.SHOW + A.RESET)
            print("\n# inspect beendet")


def _s8(v):
    return v - 256 if v >= 128 else v

def _s16(v):
    return v - 65536 if v >= 32768 else v

def _bar(val, lo, hi, width=20):
    if hi <= lo:
        return A.DIM + "─" * width + A.RESET
    frac = (val - lo) / (hi - lo)
    fill = int(round(frac * width))
    return A.GREEN + "█" * fill + A.DIM + "░" * (width - fill) + A.RESET


# ====================================================================
#  MQTT-Publisher mit Home-Assistant-Auto-Discovery
# ====================================================================
class MqttPublisher:
    """Verbindet zum Broker, meldet die Sensoren per Auto-Discovery an und
    published decodierte Werte. Verwaltet Availability pro Quelle: schläft
    ein Gerät (Votronic ohne Ladequelle), gehen seine Sensoren in HA auf
    'unavailable' statt den letzten Wert einzufrieren."""

    def __init__(self):
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            raise SystemExit("paho-mqtt fehlt:  pip install paho-mqtt")
        self._mqtt = mqtt
        self.client = mqtt.Client()
        if MQTT_USER and MQTT_USER != "DEIN_MQTT_USER":
            self.client.username_pw_set(MQTT_USER, MQTT_PASS)

        # aufeinanderfolgende Fehlschläge pro Quelle (für Availability)
        self.fail_count = {}
        # aktueller Availability-Zustand pro Quelle ("online"/"offline")
        self.avail_state = {}

        # LWT: fällt der Bridge-Prozess weg, meldet der Broker "offline"
        self.bridge_avail_topic = f"{MQTT_PREFIX}/bridge/availability"
        self.client.will_set(self.bridge_avail_topic, "offline", retain=True)

    # ---- Verbindung ----
    def connect(self):
        self.client.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
        self.client.loop_start()
        self.client.publish(self.bridge_avail_topic, "online", retain=True)
        self._publish_discovery()

    def close(self):
        try:
            self.client.publish(self.bridge_avail_topic, "offline", retain=True)
            # alle Quellen offline melden
            for src in set(d["source"] for d in SENSOR_DEFS.values()):
                self._set_availability(src, "offline", force=True)
            time.sleep(0.2)
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass

    # ---- Auto-Discovery ----
    def _avail_topic(self, source):
        return f"{MQTT_PREFIX}/{source}/availability"

    def _state_topic(self, key):
        return f"{MQTT_PREFIX}/{key}/state"

    def _publish_discovery(self):
        """Meldet jeden Sensor bei HA an. Wird retained gesendet, damit HA
        die Sensoren auch nach einem Neustart kennt."""
        device = {
            "identifiers": ["van_lin_bridge"],
            "name": "Wohnmobil LIN-Bus",
            "manufacturer": "DIY",
            "model": "ESP32 LIN Bridge",
        }
        for key, d in SENSOR_DEFS.items():
            kind = d["kind"]
            uniq = f"van_lin_{key}"
            cfg_topic = f"{DISCOVERY_PREFIX}/{kind}/{uniq}/config"
            payload = {
                "name": d["name"],
                "unique_id": uniq,
                "state_topic": self._state_topic(key),
                "availability": [
                    {"topic": self.bridge_avail_topic},
                    {"topic": self._avail_topic(d["source"])},
                ],
                "availability_mode": "all",
                "device": device,
            }
            if d.get("unit"):
                payload["unit_of_measurement"] = d["unit"]
            if d.get("dclass"):
                payload["device_class"] = d["dclass"]
            if kind == "binary_sensor":
                payload["payload_on"] = "ON"
                payload["payload_off"] = "OFF"
            self.client.publish(cfg_topic, json.dumps(payload), retain=True)

    # ---- Availability-Verwaltung ----
    def _set_availability(self, source, state, force=False):
        if force or self.avail_state.get(source) != state:
            self.avail_state[source] = state
            self.client.publish(self._avail_topic(source), state, retain=True)

    def _max_fails(self, source):
        return AVAILABILITY_MAX_FAILS.get(source, AVAILABILITY_MAX_FAILS_DEFAULT)

    def report_failure(self, source):
        """Ein Poll dieser Quelle kam ohne Antwort zurück. Erst nach genügend
        Fehlschlägen IN FOLGE wird die Quelle offline gemeldet."""
        self.fail_count[source] = self.fail_count.get(source, 0) + 1
        if self.fail_count[source] >= self._max_fails(source):
            self._set_availability(source, "offline")

    def check_timeouts(self):
        """Für Kompatibilität beibehalten — die Bewertung läuft jetzt über
        report_failure(); hier ist nichts mehr zu tun."""
        pass

    # ---- Wert-Publishing (als on_result-Callback nutzbar) ----
    def publish_result(self, name, id_byte, frame, values):
        source = name
        if frame is None or not values:
            # kein gültiger Frame -> als Fehlschlag zählen (glättet Flackern)
            self.report_failure(source)
            return
        # Quelle hat geantwortet -> Fehlerzähler zurücksetzen, online melden
        self.fail_count[source] = 0
        self._set_availability(source, "online")

        for key, val in values.items():
            if key in SKIP_KEYS or key not in SENSOR_DEFS:
                continue
            if isinstance(val, bool):
                payload = "ON" if val else "OFF"
            elif isinstance(val, float):
                payload = f"{val:.2f}"
            else:
                payload = str(val)
            self.client.publish(self._state_topic(key), payload, retain=False)


# ====================================================================
#  CLI
# ====================================================================
def cmd_scan(bridge):
    bridge.send("SCAN")
    while True:
        line = bridge.readline()
        if not line:
            continue
        if line.startswith("SCANEND"):
            print(line)
            break
        f = parse_resp(line)
        if f:
            print(f"ID 0x{f['id']:02X}  len={len(f['data'])}  "
                  f"data={' '.join(f'{b:02X}' for b in f['data'])}  "
                  f"cs={f['cs']}  -> {decode(f)}")


def cmd_read(bridge, id_hex):
    id_byte = int(id_hex, 16)
    f = bridge.read_id(id_byte)
    if f:
        print(f"ID 0x{f['id']:02X}  cs={f['cs']}  -> {decode(f)}")
    else:
        print(f"ID 0x{id_byte:02X}: keine Antwort")


def cmd_monitor(bridge):
    print("# monitor — Pi als Scheduler, Strg-C zum Beenden")
    print("# Plan: " + ", ".join(f"0x{i:02X}@{iv}s" for i, iv, _ in POLL_PLAN))
    poller = MultiPoller(bridge, POLL_PLAN, on_result=print_result)
    try:
        poller.run_forever()
    except KeyboardInterrupt:
        print("\n# stopped")


def cmd_inspect(bridge, id_hex, interval):
    id_byte = int(id_hex, 16)
    Inspector(bridge, id_byte, interval).run()


def cmd_mqtt(bridge):
    """Betriebsmodus: pollt alle IDs und published nach Home Assistant."""
    pub = MqttPublisher()
    print(f"# mqtt — verbinde zu {MQTT_HOST}:{MQTT_PORT} ...")
    pub.connect()
    print("# verbunden, Auto-Discovery gesendet. Strg-C zum Beenden.")
    print("# Plan: " + ", ".join(f"0x{i:02X}@{iv}s" for i, iv, _ in POLL_PLAN))

    poller = MultiPoller(bridge, POLL_PLAN, on_result=pub.publish_result)

    # Der MultiPoller blockiert nicht lange; wir prüfen zwischen den
    # Abfragen die Availability-Timeouts.
    last_check = 0.0
    try:
        while True:
            now = time.time()
            due = [p for p in poller.plan if now >= p["next"]]
            if due:
                due.sort(key=lambda p: p["next"])
                p = due[0]
                frame = bridge.read_id(p["id"])
                values = decode(frame) if frame else {}
                pub.publish_result(p["name"], p["id"], frame, values)
                p["next"] = now + p["interval"]
            # Timeouts ~1x/s prüfen
            if now - last_check > 1.0:
                pub.check_timeouts()
                last_check = now
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n# stopped")
    finally:
        pub.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    sub = ap.add_subparsers(dest="action", required=True)
    sub.add_parser("scan")
    p_read = sub.add_parser("read"); p_read.add_argument("id")
    sub.add_parser("monitor")
    sub.add_parser("mqtt")
    p_ins = sub.add_parser("inspect")
    p_ins.add_argument("id")
    p_ins.add_argument("--interval", type=float, default=0.5,
                       help="Poll-Intervall in Sekunden (Default 0.5)")

    args = ap.parse_args()
    bridge = LinBridge(args.port, args.baud)
    time.sleep(0.2); bridge.ser.reset_input_buffer()

    try:
        if args.action == "scan":
            cmd_scan(bridge)
        elif args.action == "read":
            cmd_read(bridge, args.id)
        elif args.action == "monitor":
            cmd_monitor(bridge)
        elif args.action == "mqtt":
            cmd_mqtt(bridge)
        elif args.action == "inspect":
            cmd_inspect(bridge, args.id, args.interval)
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
