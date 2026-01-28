# ESP32 OSC Projector Remote

ESP32-based remote for projector control over OSC (Open Sound Control) with:

- AsyncUDP for low-latency OSC reception
- ESPAsyncWebServer for web UI and OTA updates
- RS232 (Serial2) framed communication (STX/ETX) to the projector
- Event queue (FreeRTOS queue) so network callbacks only enqueue intents and the main loop performs RS232 I/O

## Files
- `esp32_async_osc.ino` — main Arduino/ESP32 sketch
- `README.md` — this file
- `LICENSE` — MIT license

## Wiring
- Serial2 TX (GPIO 17 defined as RS232_TX) -> Max3232 Module TTL RX
- Max3232 Module RS232 TX -> projector RX (DB9 Connector Pin 2)
- Serial2 RX (GPIO 16 defined as RS232_RX) -> Max3232 Module TTL TX
- Max3232 Module RS232 RX -> projector TX (DB9 Connector Pin 3)
- ESP GND -> Max3232 Module TTL-
- ESP 3v3 -> Max3232 Module TTL+
- Max3232 Module RS232- -> Projector GND (DB9 Connector Pin 5)

## Dependencies
Install these libraries (Library Manager / PlatformIO):
- AsyncTCP (ESP32)
- ESPAsyncWebServer
- AsyncUDP
- WiFiManager
- OSC (the OSCMessage library you used previously)
- Update is provided by the ESP32 core

## Usage
1. Build & flash the sketch from the Arduino IDE / PlatformIO.
2. On first boot the device will create a WiFi AP `ESP32-PROJ-SETUP` for configuration via WiFiManager.
3. Once connected, the device listens for OSC on UDP port `8000`.
   - Send `/shutter` with an integer (1=on, 0=off).
   - Send `/power` with an integer (1=on, 0=off).
4. Web UI: `http://<device-ip>/cmd` — simple buttons and debug log.
5. OTA: `http://<device-ip>/ota` — upload a new firmware binary.

## Notes
- The AsyncUDP callback is lightweight and enqueues events. RS232 writes are done in the main loop.
- Adjust `RS232_MIN_GAP_MS` if your projector needs different spacing between commands.
- Protect the OTA endpoint if you expose the device on an untrusted LAN.

## License
MIT — see LICENSE file.
