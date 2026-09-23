#include <WiFi.h>
#include <WebSocketsServer.h>
#include <SPI.h>

// =====================================================
// Wi-Fi
// =====================================================
const char* ssid = "Physics_WiFi7";
const char* password = "phy@wifi123";

// =====================================================
// RM3100 SPI pins
// =====================================================
#define SCK_PIN   18
#define MISO_PIN  19
#define MOSI_PIN  23
#define CS_PIN     5

// =====================================================
// RM3100 registers
// =====================================================
#define REG_POLL   0x00
#define REG_MX     0x24
#define REG_MY     0x27
#define REG_MZ     0x2A
#define REG_REVID  0x36

// Cycle count = 200
// Sensitivity ≈ 75 LSB/uT
#define COUNTS_PER_uT 75.0

// =====================================================
SPIClass* spi = &SPI;
WebSocketsServer webSocket(81);

// =====================================================
// RM3100 write
// =====================================================
void writeRegister(uint8_t reg, uint8_t value)
{
  digitalWrite(CS_PIN, LOW);

  spi->transfer(reg);
  spi->transfer(value);

  digitalWrite(CS_PIN, HIGH);
}

// =====================================================
// RM3100 read
// =====================================================
void readRegisters(uint8_t reg, uint8_t* data, uint8_t length)
{
  digitalWrite(CS_PIN, LOW);

  spi->transfer(reg | 0x80);

  for (uint8_t i = 0; i < length; i++)
  {
    data[i] = spi->transfer(0x00);
  }

  digitalWrite(CS_PIN, HIGH);
}

// =====================================================
// Read 24-bit signed value
// =====================================================
int32_t read24(uint8_t reg)
{
  uint8_t data[3];

  readRegisters(reg, data, 3);

  int32_t value =
      ((int32_t)data[0] << 16) |
      ((int32_t)data[1] << 8) |
      data[2];

  if (value & 0x800000)
  {
    value |= 0xFF000000;
  }

  return value;
}

// =====================================================
// WebSocket event handler
// =====================================================
void webSocketEvent(
  uint8_t clientNum,
  WStype_t type,
  uint8_t* payload,
  size_t length)
{
  switch (type)
  {
    case WStype_CONNECTED:
    {
      Serial.print("WebSocket client connected: ");
      Serial.println(clientNum);
      break;
    }

    case WStype_DISCONNECTED:
    {
      Serial.print("WebSocket client disconnected: ");
      Serial.println(clientNum);
      break;
    }

    case WStype_TEXT:
    {
      Serial.print("Received: ");
      Serial.println((char*)payload);
      break;
    }

    default:
      break;
  }
}

// =====================================================
// Setup
// =====================================================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("================================");
  Serial.println("ESP32 + RM3100 + WEBSOCKET");
  Serial.println("================================");

  // -------------------------------
  // SPI setup
  // -------------------------------
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  spi->begin(
    SCK_PIN,
    MISO_PIN,
    MOSI_PIN,
    CS_PIN
  );

  // -------------------------------
  // RM3100 revision
  // -------------------------------
  spi->beginTransaction(
    SPISettings(1000000, MSBFIRST, SPI_MODE0)
  );

  uint8_t revision;

  readRegisters(
    REG_REVID,
    &revision,
    1
  );

  spi->endTransaction();

  Serial.print("RM3100 Revision ID: 0x");
  Serial.println(revision, HEX);

  // -------------------------------
  // Cycle count = 200
  // -------------------------------
  spi->beginTransaction(
    SPISettings(1000000, MSBFIRST, SPI_MODE0)
  );

  // X = 200
  writeRegister(0x04, 0x00);
  writeRegister(0x05, 0xC8);

  // Y = 200
  writeRegister(0x06, 0x00);
  writeRegister(0x07, 0xC8);

  // Z = 200
  writeRegister(0x08, 0x00);
  writeRegister(0x09, 0xC8);

  spi->endTransaction();

  Serial.println("RM3100 configured.");

  // =================================================
  // Wi-Fi
  // =================================================

  Serial.println();
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Wi-Fi connected!");

  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // =================================================
  // WebSocket server
  // =================================================

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("WebSocket server started.");
  Serial.print("WebSocket URL: ws://");
  Serial.print(WiFi.localIP());
  Serial.println(":81");

  Serial.println();
  Serial.println("Starting measurements...");
}

// =====================================================
// Loop
// =====================================================
void loop()
{
  // Keep WebSocket alive
  webSocket.loop();

  // =================================================
  // Start RM3100 measurement
  // =================================================

  spi->beginTransaction(
    SPISettings(1000000, MSBFIRST, SPI_MODE0)
  );

  writeRegister(REG_POLL, 0x70);

  spi->endTransaction();

  delay(100);

  // =================================================
  // Read X/Y/Z
  // =================================================

  spi->beginTransaction(
    SPISettings(1000000, MSBFIRST, SPI_MODE0)
  );

  int32_t mx = read24(REG_MX);
  int32_t my = read24(REG_MY);
  int32_t mz = read24(REG_MZ);

  spi->endTransaction();

  // =================================================
  // Convert to microtesla
  // =================================================

  float Bx = mx / COUNTS_PER_uT;
  float By = my / COUNTS_PER_uT;
  float Bz = mz / COUNTS_PER_uT;

  // =================================================
  // Serial output
  // =================================================

  Serial.print("Bx = ");
  Serial.print(Bx, 3);

  Serial.print(" uT   By = ");
  Serial.print(By, 3);

  Serial.print(" uT   Bz = ");
  Serial.print(Bz, 3);

  Serial.println(" uT");

  // =================================================
  // JSON packet
  // =================================================

  String json = "{";
  json += "\"Bx\":";
  json += String(Bx, 3);
  json += ",";
  json += "\"By\":";
  json += String(By, 3);
  json += ",";
  json += "\"Bz\":";
  json += String(Bz, 3);
  json += "}";

  // =================================================
  // Send to all WebSocket clients
  // =================================================

  webSocket.broadcastTXT(json);

  delay(100);
}
