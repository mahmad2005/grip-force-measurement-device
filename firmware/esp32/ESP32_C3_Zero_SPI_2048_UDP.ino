#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>

// SPI pins
#define PIN_MOSI 4
#define PIN_MISO 3
#define PIN_CLK  2
#define PIN_CS   1

#define PIN_IMU_CS 5

// Wi-Fi Credentials
const char* ssid = "uofrGuest";
const char* password = "";
//const char* ssid = "SASKTEL023Y";  //  Melville
//const char* password = "Homescotia5066";

// UDP Settings
 const char* udpAddress = "10.69.167.5"; //Lab IP  // <-- Your PC's local IP address 
//const char* udpAddress = "172.16.1.180"; // Melville House
const int udpPort = 12345; // Target UDP port on PC

WiFiUDP udp;

// SPI & Transfer Config
const int TOTAL_VALUES = 2048;  // 32x64
const int VALUES_PER_CHUNK = 205;  // 205 x 2 = 410 bytes
const int CHUNK_COUNT = 10;

uint16_t readings[TOTAL_VALUES];
uint8_t reading_who_im;

void setup() {
  delay(1000);
  Serial.begin(115200);
  SPI.begin(PIN_CLK, PIN_MISO, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  pinMode(PIN_IMU_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);

  // Wi-Fi connect
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected!");

  udp.begin(WiFi.localIP(), udpPort);
  Serial.println("UDP client ready.");
}

void loop() {
  // 1. SPI: read 2048 uint16_t = 4096 bytes
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(10);
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0)); // 400000

  for (int i = 0; i < TOTAL_VALUES; i++) {
    readings[i] = SPI.transfer16(0x0000);  // dummy send to receive
  }

  SPI.endTransaction();
  digitalWrite(PIN_CS, HIGH);

  // 2. Send as 10 UDP chunks of 410 bytes each (205 x uint16_t)
  uint8_t udp_packet[415];  // 2-byte start + 1 index + 410 data + 2-byte end

  for (int chunk = 0; chunk < CHUNK_COUNT; chunk++) {
    int start_index = chunk * VALUES_PER_CHUNK;

    // Header
    udp_packet[0] = 0xAA;
    udp_packet[1] = 0x55;
    udp_packet[2] = chunk;

    // Data
    for (int i = 0; i < VALUES_PER_CHUNK; i++) {
      int data_index = start_index + i;
      uint16_t val = (data_index < TOTAL_VALUES) ? readings[data_index] : 0; // pad with 0 if overflow

      udp_packet[3 + i * 2]     = val & 0xFF;       // LSB
      udp_packet[3 + i * 2 + 1] = (val >> 8) & 0xFF; // MSB
    }

    // Footer
    udp_packet[3 + VALUES_PER_CHUNK * 2] = 0x55;
    udp_packet[4 + VALUES_PER_CHUNK * 2] = 0xAA;

    // Send
    udp.beginPacket(udpAddress, udpPort);
    udp.write(udp_packet, sizeof(udp_packet));
    udp.endPacket();

    delayMicroseconds(100);

  // 3. SPI: read ICM-20948
  digitalWrite(PIN_IMU_CS, LOW);
  delayMicroseconds(10);
  //SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

 // for (int i = 0; i < TOTAL_VALUES; i++) {
 //   readings[i] = SPI.transfer16(0x0000);  // dummy send to receive
 // }
  //uint8_t reg = 0x00;
  //reading_who_im = SPI.transfer(reg | 0x80);

  // Send register address (0x00 | 0x80 for read)
  SPI.transfer(0x00 | 0x80);

  // Read response (WHO_AM_I value)
  reading_who_im = SPI.transfer(0x00);  // Send dummy, receive data

  SPI.endTransaction();
  digitalWrite(PIN_IMU_CS, HIGH);
  }
  Serial.println(reading_who_im);

 //Serial.println("Sent full frame as 10 chunks.\n");
  //delayMicroseconds(100);
  delay(50);
}