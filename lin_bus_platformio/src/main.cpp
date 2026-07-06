// =====================================================================
//  LIN Bridge für ESP32 D1 Mini + WomoLIN (TJA1021T)
//  Variante C: ESP32 = zeitkritisches LIN-Frontend, Decodierung am Pi.
//
//  Kommandoprotokoll (zeilenbasiert, über USB-Seriell @115200):
//    SCAN                -> alle IDs 0x00..0x3F einmal abfragen
//    READ <id>           -> eine ID einmal pollen        (id hex, z.B. 0C)
//    POLL <id> <ms>      -> ID zyklisch pollen alle <ms>  (STOP beendet)
//    STOP                -> laufenden POLL beenden
//    BAUD <n>            -> LIN-Baudrate umstellen (z.B. 9600 / 19200)
//    HELP                -> Befehlsliste
//
//  Antwortzeilen (maschinen- und menschenlesbar):
//    RESP <id> <pid> <len> <b0..bn> <crc> <classic|enh|bad>
//    EMPTY <id>          -> keine Antwort
//    OK <text>           -> Bestätigung / Info
//    ERR <text>          -> Fehler / ungültiges Kommando
//    SCANEND <count>     -> Scan fertig, Anzahl antwortender IDs
//  Alle Frame-Bytes als 2-stelliges HEX, großgeschrieben.
// =====================================================================

#include <Arduino.h>

// ---------- Konfiguration ----------
#define LIN_SERIAL     Serial1
#define LIN_RX_PIN     16          // an WomoLIN TX
#define LIN_TX_PIN     17          // an WomoLIN RX
#define CMD_SERIAL     Serial      // USB zum Pi
#define CMD_BAUD       115200
#define RESP_TO_MS     30          // Wartezeit auf Slave-Antwort
#define ECHO_BYTES     2           // Sync(0x55)+PID werden mitgehört -> überspringen
#define MAX_FRAME      16

// ---------- Laufzeitzustand ----------
uint32_t g_linBaud   = 19200;      // per BAUD-Kommando änderbar
bool     g_polling   = false;
uint8_t  g_pollId    = 0;
uint32_t g_pollMs    = 500;
uint32_t g_lastPoll  = 0;

// ---------- LIN-Helfer ----------
uint8_t linPID(uint8_t id) {
  id &= 0x3F;
  uint8_t p0 = ((id>>0)^(id>>1)^(id>>2)^(id>>4)) & 1;
  uint8_t p1 = ~((id>>1)^(id>>3)^(id>>4)^(id>>5)) & 1;
  return id | (p0<<6) | (p1<<7);
}

uint8_t linChecksum(uint8_t *d, int n, uint8_t pid, bool enhanced) {
  uint16_t s = enhanced ? pid : 0;
  for (int i = 0; i < n; i++) { s += d[i]; if (s > 0xFF) s -= 0xFF; }
  return (uint8_t)(~s);
}

void linBegin(uint32_t baud) {
  LIN_SERIAL.begin(baud, SERIAL_8N1, LIN_RX_PIN, LIN_TX_PIN);
}

// Break durch kurzes Umschalten auf halbe Baudrate + 0x00
void sendBreak() {
  LIN_SERIAL.flush();
  LIN_SERIAL.begin(g_linBaud/2, SERIAL_8N1, LIN_RX_PIN, LIN_TX_PIN);
  LIN_SERIAL.write((uint8_t)0x00);
  LIN_SERIAL.flush();
  LIN_SERIAL.begin(g_linBaud, SERIAL_8N1, LIN_RX_PIN, LIN_TX_PIN);
}

void sendHeader(uint8_t id) {
  sendBreak();
  LIN_SERIAL.write(0x55);
  LIN_SERIAL.write(linPID(id));
  LIN_SERIAL.flush();
}

// Fragt eine ID ab, füllt data[]/len, gibt true bei Antwort.
// crc wird separat zurückgegeben, csState: 0=bad 1=classic 2=enhanced
bool queryID(uint8_t id, uint8_t *data, int &len, uint8_t &crc, int &csState) {
  while (LIN_SERIAL.available()) LIN_SERIAL.read();   // RX leeren
  sendHeader(id);

  uint8_t buf[MAX_FRAME]; int n = 0;
  uint32_t t0 = millis();
  while (millis()-t0 < RESP_TO_MS && n < MAX_FRAME) {
    if (LIN_SERIAL.available()) { buf[n++] = LIN_SERIAL.read(); t0 = millis(); }
  }

  if (n <= ECHO_BYTES + 1) { len = 0; return false; }  // nur Echo o. nichts

  len = n - ECHO_BYTES - 1;        // Echo abziehen, letztes Byte = CRC
  for (int i = 0; i < len; i++) data[i] = buf[ECHO_BYTES + i];
  crc = buf[n-1];

  uint8_t pid = linPID(id);
  if      (linChecksum(data, len, pid, true)  == crc) csState = 2;
  else if (linChecksum(data, len, pid, false) == crc) csState = 1;
  else                                                csState = 0;
  return true;
}

void printFrame(uint8_t id) {
  uint8_t data[MAX_FRAME]; int len; uint8_t crc; int cs;
  if (queryID(id, data, len, crc, cs)) {
    CMD_SERIAL.printf("RESP %02X %02X %d", id, linPID(id), len);
    for (int i = 0; i < len; i++) CMD_SERIAL.printf(" %02X", data[i]);
    CMD_SERIAL.printf(" %02X %s\n", crc,
      cs==2 ? "enh" : cs==1 ? "classic" : "bad");
  } else {
    CMD_SERIAL.printf("EMPTY %02X\n", id);
  }
}

void doScan() {
  CMD_SERIAL.println("OK scan start");
  int found = 0;
  for (uint16_t id = 0x00; id <= 0x3F; id++) {
    uint8_t data[MAX_FRAME]; int len; uint8_t crc; int cs;
    // zwei Versuche pro ID, träge Slaves wollen manchmal einen zweiten Header
    bool got = queryID(id, data, len, crc, cs);
    if (!got) { delay(5); got = queryID(id, data, len, crc, cs); }
    if (got) {
      found++;
      CMD_SERIAL.printf("RESP %02X %02X %d", id, linPID(id), len);
      for (int i = 0; i < len; i++) CMD_SERIAL.printf(" %02X", data[i]);
      CMD_SERIAL.printf(" %02X %s\n", crc,
        cs==2 ? "enh" : cs==1 ? "classic" : "bad");
    }
    delay(10);
  }
  CMD_SERIAL.printf("SCANEND %d\n", found);
}

// ---------- Kommandoverarbeitung ----------
int parseHexByte(const String &s, bool &ok) {
  ok = false;
  if (s.length() == 0 || s.length() > 2) return 0;
  int v = (int) strtol(s.c_str(), nullptr, 16);
  if (v < 0 || v > 0xFF) return 0;
  ok = true;
  return v;
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  // in Token zerlegen
  String cmd = line;
  String a1 = "", a2 = "";
  int sp1 = line.indexOf(' ');
  if (sp1 >= 0) {
    cmd = line.substring(0, sp1);
    String rest = line.substring(sp1 + 1); rest.trim();
    int sp2 = rest.indexOf(' ');
    if (sp2 >= 0) { a1 = rest.substring(0, sp2); a2 = rest.substring(sp2+1); a2.trim(); }
    else          { a1 = rest; }
  }
  cmd.toUpperCase();

  if (cmd == "SCAN") {
    g_polling = false;
    doScan();
  }
  else if (cmd == "READ") {
    bool ok; int id = parseHexByte(a1, ok);
    if (!ok) { CMD_SERIAL.println("ERR read needs hex id, e.g. READ 0C"); return; }
    printFrame((uint8_t)id);
  }
  else if (cmd == "POLL") {
    bool ok; int id = parseHexByte(a1, ok);
    if (!ok) { CMD_SERIAL.println("ERR poll needs hex id, e.g. POLL 16 500"); return; }
    uint32_t ms = a2.length() ? (uint32_t) a2.toInt() : 500;
    if (ms < 20) ms = 20;
    g_pollId = (uint8_t) id; g_pollMs = ms;
    g_polling = true; g_lastPoll = 0;
    CMD_SERIAL.printf("OK polling %02X every %lu ms (STOP to end)\n", g_pollId, (unsigned long)ms);
  }
  else if (cmd == "STOP") {
    g_polling = false;
    CMD_SERIAL.println("OK stopped");
  }
  else if (cmd == "BAUD") {
    uint32_t b = (uint32_t) a1.toInt();
    if (b < 1000 || b > 250000) { CMD_SERIAL.println("ERR baud out of range"); return; }
    g_linBaud = b;
    linBegin(g_linBaud);
    CMD_SERIAL.printf("OK lin baud %lu\n", (unsigned long)g_linBaud);
  }
  else if (cmd == "HELP") {
    CMD_SERIAL.println("OK commands:");
    CMD_SERIAL.println("  SCAN            scan all ids 00..3F");
    CMD_SERIAL.println("  READ <id>       poll one id once   (hex, e.g. READ 0C)");
    CMD_SERIAL.println("  POLL <id> <ms>  poll id repeatedly  (e.g. POLL 16 500)");
    CMD_SERIAL.println("  STOP            stop repeated poll");
    CMD_SERIAL.println("  BAUD <n>        set lin baud (9600 / 19200)");
    CMD_SERIAL.println("  HELP            this list");
  }
  else {
    CMD_SERIAL.printf("ERR unknown command: %s\n", cmd.c_str());
  }
}

// ---------- Setup / Loop ----------
String g_inbuf;

void setup() {
  CMD_SERIAL.begin(CMD_BAUD);
  delay(300);
  linBegin(g_linBaud);
  CMD_SERIAL.println("OK lin-bridge ready");
  CMD_SERIAL.printf("OK lin baud %lu, type HELP\n", (unsigned long)g_linBaud);
}

void loop() {
  // Kommandos zeilenweise einlesen
  while (CMD_SERIAL.available()) {
    char c = (char) CMD_SERIAL.read();
    if (c == '\n' || c == '\r') {
      if (g_inbuf.length()) { handleCommand(g_inbuf); g_inbuf = ""; }
    } else {
      g_inbuf += c;
      if (g_inbuf.length() > 64) g_inbuf = "";  // Überlauf verwerfen
    }
  }

  // zyklisches Pollen
  if (g_polling && (millis() - g_lastPoll >= g_pollMs)) {
    g_lastPoll = millis();
    printFrame(g_pollId);
  }
}