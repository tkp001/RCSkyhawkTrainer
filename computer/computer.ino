#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);
const uint64_t pipeTX = 0xF0F0F0F0E1LL; // Computer TX -> Plane RX
const uint64_t pipeRX = 0xF0F0F0F0D2LL; // Computer RX <- Plane TX

struct PythonPacket {
  uint8_t autostabilize;
  uint8_t throttle;      // 0-100
  int8_t aileronL;       // -90..90
  int8_t aileronR;
  int8_t elevator;
  int8_t rudder;
  uint8_t checksum;
};

struct PythonRecievePacket {
  float roll, pitch, yaw;
  float voltage;
  uint8_t throttleActual;
  int8_t aileronLActual;
  int8_t aileronRActual;
  int8_t elevatorActual;
  int8_t rudderActual;
  uint8_t checksum;
};

struct ControlPacket {
  uint16_t seq;          // sequence number from computer
  uint8_t autostabilize;
  uint8_t throttle;      // 0-100
  int8_t aileronL;       // -90..90
  int8_t aileronR;
  int8_t elevator;
  int8_t rudder;
  uint8_t checksum;
};

struct TelemetryPacket {
  uint16_t seq;          // sequence number from plane
  float roll, pitch, yaw;
  float voltage;
  uint8_t throttleActual;
  int8_t aileronLActual;
  int8_t aileronRActual;
  int8_t elevatorActual;
  int8_t rudderActual;
  uint8_t checksum;
};

uint8_t calculateChecksum(uint8_t *data, size_t length) {
  uint8_t sum = 0;
  for (size_t i = 0; i < length; i++) sum ^= data[i];
  return sum;
}

ControlPacket control;
TelemetryPacket telemetry;
PythonPacket receivedPythonPacket;
PythonRecievePacket sentArduinoPacket;

unsigned long lastSendTime = 0;
unsigned long lastRecvTime = 0;
unsigned long lastPythonDataTime = 0;  // NEW: Track when we last got Python data
const unsigned long interval = 50;  // 20 Hz
const unsigned long pythonTimeout = 200;  // NEW: 200ms timeout for fresh Python data

uint16_t txSeq = 0;
uint16_t lastRxSeq = 0;
bool hasFreshPythonData = false;  // NEW: Flag to track if we have fresh data

void setup() {
  Serial.begin(115200);

  radio.begin();
  radio.setChannel(90);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipeTX);
  radio.openReadingPipe(1, pipeRX);
  radio.startListening();

  //control packet init
  control.autostabilize = 0;
  control.throttle = 0;
  control.aileronL = 0;
  control.aileronR = 0;
  control.elevator = 0;
  control.rudder = 0;

  //telemetry packet init
  sentArduinoPacket.roll = 0.0;
  sentArduinoPacket.pitch = 0.0;
  sentArduinoPacket.yaw = 0.0;
  sentArduinoPacket.voltage = 0.0;
  sentArduinoPacket.throttleActual = 0;
  sentArduinoPacket.aileronLActual = 0;
  sentArduinoPacket.aileronRActual = 0;
  sentArduinoPacket.elevatorActual = 0;
  sentArduinoPacket.rudderActual = 0;
  sentArduinoPacket.checksum = 0;

  Serial.println("SETUP DONE.");
}

void loop() {
  unsigned long now = millis();

  // Always check for incoming Python data
  if (Serial.available() >= sizeof(PythonPacket)) {
    size_t bytesRead = Serial.readBytes((char*)&receivedPythonPacket, sizeof(PythonPacket));

    if (bytesRead == sizeof(PythonPacket)) {
      uint8_t receivedChecksum = receivedPythonPacket.checksum;
      uint8_t calculatedChecksum = calculateChecksum(
          (uint8_t*)&receivedPythonPacket,
          sizeof(PythonPacket) - sizeof(receivedPythonPacket.checksum)
      );

      if (calculatedChecksum == receivedChecksum) {
        // NEW: Update control data and mark as fresh
        control.autostabilize = receivedPythonPacket.autostabilize;
        control.throttle = receivedPythonPacket.throttle;
        control.aileronL = receivedPythonPacket.aileronL;
        control.aileronR = receivedPythonPacket.aileronR;
        control.elevator = receivedPythonPacket.elevator;
        control.rudder = receivedPythonPacket.rudder;
        
        lastPythonDataTime = now;  // NEW: Record when we got fresh data
        hasFreshPythonData = true; // NEW: Mark data as fresh
        
      } else {
        // Clear the serial buffer on checksum error
        while(Serial.available()) {
          Serial.read();
        }
      }
    } else {
      // Clear buffer on incomplete packet
      while(Serial.available()) {
        Serial.read();
      }
    }
  }

  // NEW: Check if Python data has timed out
  if (hasFreshPythonData && (now - lastPythonDataTime > pythonTimeout)) {
    hasFreshPythonData = false;  // Mark data as stale
  }

  // Send control packet every interval - but only if we have fresh Python data
  if (now - lastSendTime >= interval) {
    lastSendTime = now;

    // NEW: Only transmit if we have fresh Python data
    if (hasFreshPythonData) {
      control.seq = txSeq++;
      control.checksum = calculateChecksum((uint8_t*)&control, sizeof(control) - 1);

      radio.stopListening();
      bool ok = radio.write(&control, sizeof(control));
      radio.startListening();

      // Optional debug output
      // Serial.print("TX Seq: ");
      // Serial.print(control.seq);
      // Serial.println(ok ? " TX OK" : " TX FAIL");
    } else {
      // Optional debug - indicate we're not transmitting
      // Serial.println("No fresh Python data - not transmitting");
    }
  }

  // Always check for incoming telemetry (unchanged)
  if (radio.available()) {
    radio.read(&telemetry, sizeof(telemetry));

    uint8_t cs = calculateChecksum((uint8_t*)&telemetry, sizeof(telemetry) - 1);
    if (cs == telemetry.checksum) {
      lastRecvTime = now;
      lastRxSeq = telemetry.seq;

      // Send telemetry to Python
      sentArduinoPacket.roll = telemetry.roll;
      sentArduinoPacket.pitch = telemetry.pitch;
      sentArduinoPacket.yaw = telemetry.yaw;
      sentArduinoPacket.voltage = telemetry.voltage;
      sentArduinoPacket.throttleActual = telemetry.throttleActual;
      sentArduinoPacket.aileronLActual = telemetry.aileronLActual;
      sentArduinoPacket.aileronRActual = telemetry.aileronRActual;
      sentArduinoPacket.elevatorActual = telemetry.elevatorActual;
      sentArduinoPacket.rudderActual = telemetry.rudderActual;

      uint8_t calculated_checksum_send = calculateChecksum(
          (uint8_t*)&sentArduinoPacket,
          sizeof(PythonRecievePacket) - sizeof(sentArduinoPacket.checksum)
      );
      sentArduinoPacket.checksum = calculated_checksum_send;

      Serial.write((uint8_t*)&sentArduinoPacket, sizeof(sentArduinoPacket));
    }
  }
}