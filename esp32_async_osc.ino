/*
  ESP32 Async OSC + RS232 + OTA + Event Queue

  - AsyncUDP for OSC packets (low latency)
  - AsyncWebServer for web UI and OTA (non-blocking)
  - RS232 framed TX/RX using STX/ETX on Serial2
  - Event queue (FreeRTOS) so UDP callback only enqueues events and main loop processes RS232 sends
*/

#include <WiFi.h>
#include <WiFiManager.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncUDP.h>

#include <OSCMessage.h>
#include <Update.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ---------- CONFIG ----------
const int OSC_PORT = 8000;

// UART RS232 pins
#define RS232_TX 17
#define RS232_RX 16

// DEBUG
#define DEBUG 1

#if DEBUG
  #define DBG(x)   Serial.print(x)
  #define DBGL(x)  Serial.println(x)
#else
  #define DBG(x)
  #define DBGL(x)
#endif

#define LOG_LINES 12
String logBuf[LOG_LINES];
int logIdx = 0;

// STX/ETX framing for RS232
#define STX 0x02
#define ETX 0x03

// inter-command minimum spacing (ms)
const unsigned long RS232_MIN_GAP_MS = 500;
unsigned long nextTXtime = 0;

// Async server & udp
AsyncWebServer server(80);
AsyncUDP asyncUdp;

// saved logical states
bool shutterState = false;
bool powerState   = false;

// ---------- Event queue types ----------
typedef enum {
  EVT_SHUTTER = 0,
  EVT_POWER
} EventType;

typedef struct {
  EventType type;
  bool value;
} Event;

static QueueHandle_t eventQueue = NULL;
const size_t EVENT_QUEUE_LEN = 16; // capacity

void logweb(const String& s) {
  String entry = String(millis()) + "ms: " + s;
  logBuf[logIdx] = entry;
  logIdx = (logIdx+1) % LOG_LINES;
  DBGL(entry);
}

// low-level framed RS232 transmit
void sendCommandRS232(const char* id, const char* cmd, const char* param = "") {
  // wait until it's OK to send
  while (millis() < nextTXtime) delay(1);

  Serial2.write((uint8_t)STX);
  Serial2.print("AD");
  Serial2.print(id);
  Serial2.print(';');
  Serial2.print(cmd);
  if (strlen(param) > 0) {
    Serial2.print(':');
    Serial2.print(param);
  }
  Serial2.write((uint8_t)ETX);

  // update nextTXtime to enforce spacing between commands
  nextTXtime = millis() + RS232_MIN_GAP_MS;
}

void sendShutter(bool on) {
  shutterState = on;
  logweb(String("Processing event -> RS232 TX: OSH:") + (on ? "1" : "0"));
  sendCommandRS232("ZZ", "OSH", on ? "1" : "0");
}

void sendPower(bool on) {
  powerState = on;
  logweb(String("Processing event -> RS232 TX: ") + (on ? "PON" : "POF"));
  if (on) sendCommandRS232("ZZ", "PON");
  else    sendCommandRS232("ZZ", "POF");
}

// RS232 receive: collect framed messages between STX and ETX
void pollRS232() {
  static bool inFrame = false;
  static String frame;
  while (Serial2.available()) {
    int v = Serial2.read();
    if (v < 0) break;
    uint8_t c = (uint8_t)v;

    if (c == STX) {
      inFrame = true;
      frame = "";
      continue;
    }
    if (c == ETX) {
      if (inFrame) {
        logweb(String("RS232 RX FRAME: ") + frame);
        // TODO: parse device responses if needed
      }
      inFrame = false;
      frame = "";
      continue;
    }
    if (inFrame) {
      frame += (char)c;
    } else {
      // optionally log stray bytes (commented out)
      // logweb(String("RS232 RX stray: ") + (char)c);
    }
  }
  // (do not artificially modify nextTXtime here)
}

// ---------- OSC handlers (enqueue events, minimal work in callback) ----------
void onShutterOSC(OSCMessage &msg) {
  if (!eventQueue) return;
  if (msg.isInt(0)) {
    Event e;
    e.type = EVT_SHUTTER;
    e.value = (msg.getInt(0) == 1);
    if (xQueueSend(eventQueue, &e, 0) != pdTRUE) {
      // queue full; log minimal info (avoid heavy String ops in callback)
      DBGL("Event queue full (shutter)");
    }
  }
}

void onPowerOSC(OSCMessage &msg) {
  if (!eventQueue) return;
  if (msg.isInt(0)) {
    Event e;
    e.type = EVT_POWER;
    e.value = (msg.getInt(0) == 1);
    if (xQueueSend(eventQueue, &e, 0) != pdTRUE) {
      DBGL("Event queue full (power)");
    }
  }
}

// web UI page (simple)
String cmdPage() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width'>";
  html += "<style>body{font-family:sans-serif}button{font-size:18px;margin:6px;padding:8px}</style>";
  html += "</head><body>";
  html += "<h2>Projecteur</h2>";

  html += "<p>Shutter: ";
  html += shutterState ? "ON" : "OFF";
  html += "</p>";
  html += "<a href='/shutter?set=1'><button>Shutter ON</button></a>";
  html += "<a href='/shutter?set=0'><button>Shutter OFF</button></a>";

  html += "<p>Power: ";
  html += powerState ? "ON" : "OFF";
  html += "</p>";
  html += "<a href='/power?set=1'><button>Power ON</button></a>";
  html += "<a href='/power?set=0'><button>Power OFF</button></a>";

  html += "<h3>Debug</h3><pre>";
  for (int i = 0; i < LOG_LINES; i++) {
    int idx = (logIdx + i) % LOG_LINES; // oldest-first
    if (logBuf[idx].length()) html += logBuf[idx] + "\n";
  }
  html += "</pre></body></html>";
  return html;
}

const char* otaPage =
"<html><body>"
"<h2>ESP32 OTA Update</h2>"
"<form method='POST' action='/ota' enctype='multipart/form-data'>"
"<input type='file' name='firmware'>"
"<input type='submit' value='Update'>"
"</form>"
"</body></html>";

// ---------- Setup web endpoints (async) ----------
void setupAsyncWeb() {
  // GET /cmd - UI
  server.on("/cmd", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", cmdPage());
  });

  // shutter endpoint
  server.on("/shutter", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("set")) {
      String s = request->getParam("set")->value();
      // enqueue instead of direct send, to unify processing path
      if (eventQueue) {
        Event e;
        e.type = EVT_SHUTTER;
        e.value = (s == "1");
        if (xQueueSend(eventQueue, &e, 0) != pdTRUE) {
          DBGL("Event queue full (web shutter)");
        }
      } else {
        // fallback
        sendShutter(s == "1");
      }
    }
    // redirect back
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->setCode(303);
    response->addHeader("Location", "/cmd");
    request->send(response);
  });

  // power endpoint
  server.on("/power", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("set")) {
      String s = request->getParam("set")->value();
      if (eventQueue) {
        Event e;
        e.type = EVT_POWER;
        e.value = (s == "1");
        if (xQueueSend(eventQueue, &e, 0) != pdTRUE) {
          DBGL("Event queue full (web power)");
        }
      } else {
        sendPower(s == "1");
      }
    }
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->setCode(303);
    response->addHeader("Location", "/cmd");
    request->send(response);
  });

  // OTA page GET
  server.on("/ota", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", otaPage);
  });

  // OTA upload handler (async)
  server.on("/ota", HTTP_POST,
    // onRequest (finished)
    [](AsyncWebServerRequest *request) {
      if (Update.hasError()) {
        request->send(500, "text/plain", "OTA Update FAILED");
      } else {
        request->send(200, "text/plain", "OTA Update OK. Rebooting...");
        delay(500);
        ESP.restart();
      }
    },
    // onUpload - called repeatedly with file chunks
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      if (index == 0) {
        DBGL("OTA: start");
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          DBGL("OTA: Update.begin failed");
        }
      }
      if (len) {
        Update.write(data, len);
      }
      if (final) {
        bool ok = Update.end(true);
        DBGL(String("OTA: end => ") + (ok ? "OK" : "FAILED"));
      }
    }
  );

  server.begin();
}

// ---------- Setup WiFi (WiFiManager) ----------
void setupWiFi() {
  WiFiManager wm;
  wm.setDebugOutput(DEBUG);
  wm.setTimeout(180);

  // Start AP if not configured. SSID: "ESP32-PROJ-SETUP"
  if (!wm.autoConnect("ESP32-PROJ-SETUP")) {
    DBGL("WiFi setup failed, restarting...");
    ESP.restart();
  } else {
    DBGL(String("Connected, IP: ") + WiFi.localIP().toString());
  }
}

// ---------- Setup AsyncUDP for OSC ----------
bool setupAsyncOSC() {
  if (asyncUdp.listen(OSC_PORT)) {
    DBGL(String("AsyncUDP listening on port ") + OSC_PORT);
    asyncUdp.onPacket([](AsyncUDPPacket packet) {
      // very low-latency handler
      // copy packet into OSCMessage
      OSCMessage msg;
      const uint8_t *data = packet.data();
      size_t len = packet.length();
      for (size_t i = 0; i < len; ++i) msg.fill(data[i]);

      if (!msg.hasError()) {
        // dispatch - handlers enqueuing events (lightweight)
        msg.dispatch("/shutter", onShutterOSC);
        msg.dispatch("/power", onPowerOSC);
      } else {
        // avoid heavy String ops here; use a light log
        DBGL("OSC parse error in UDP callback");
      }
    });
    return true;
  } else {
    DBGL("AsyncUDP listen failed");
    return false;
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(100);

  // create event queue
  eventQueue = xQueueCreate(EVENT_QUEUE_LEN, sizeof(Event));
  if (!eventQueue) {
    DBGL("Failed to create event queue");
  } else {
    DBGL(String("Event queue created, length=") + String(EVENT_QUEUE_LEN));
  }

  // use RS232 defines explicitly
  Serial2.begin(9600, SERIAL_8N1, RS232_RX, RS232_TX);

  setupWiFi();
  setupAsyncWeb();

  if (!setupAsyncOSC()) {
    DBGL("Warning: AsyncOSC not running");
  }

  DBGL("System ready");
}

// ---------- LOOP ----------
void loop() {
  // server is async; no need to call handleClient
  pollRS232();

  // Drain and process all pending events (non-blocking)
  Event e;
  while (eventQueue && xQueueReceive(eventQueue, &e, 0) == pdTRUE) {
    switch (e.type) {
      case EVT_SHUTTER:
        sendShutter(e.value);
        break;
      case EVT_POWER:
        sendPower(e.value);
        break;
      default:
        DBGL("Unknown event type");
    }
  }

  // You can do other housekeeping here
  delay(1);
}
