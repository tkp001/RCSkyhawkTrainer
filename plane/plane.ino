#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include "MadgwickAHRS.h"
#include <avr/wdt.h>

RF24 radio(9, 10);
const uint64_t pipeTX = 0xF0F0F0F0D2LL; // Plane TX -> Computer RX
const uint64_t pipeRX = 0xF0F0F0F0E1LL; // Plane RX <- Computer TX
const int voltageSamples = 10;
uint8_t mcusr_status;

Servo servoAilL, servoAilR, servoElev, servoRudd, esc;
MPU6050 mpu;
Madgwick filter;

struct ControlPacket {
  uint16_t seq;
  uint8_t autostabilize;
  uint8_t throttle;
  int8_t aileronL;
  int8_t aileronR;
  int8_t elevator;
  int8_t rudder;
  uint8_t checksum;
};

struct TelemetryPacket {
  uint16_t seq;
  float roll, pitch, yaw;
  float voltage;
  uint8_t throttleActual;
  int8_t aileronLActual;
  int8_t aileronRActual;
  int8_t elevatorActual;
  int8_t rudderActual;
  uint8_t checksum;
};

struct PIDController {
  float kP, kI, kD;
  float integral;
  float previousError;
  float output;
  unsigned long lastTime;
};

uint8_t calculateChecksum(uint8_t *data, size_t length) {
  uint8_t sum = 0;
  for (size_t i = 0; i < length; i++) sum ^= data[i];
  return sum;
}

ControlPacket control, lastGoodControl;
TelemetryPacket telemetry;

PIDController rollPID = {2.0, 0.1, 0.5, 0, 0, 0, 0};    // Roll stabilization
PIDController pitchPID = {2.0, 0.1, 0.5, 0, 0, 0, 0};   // Pitch stabilization
PIDController yawPID = {1.5, 0.05, 0.3, 0, 0, 0, 0};    // Yaw stabilization

// Target angles (setpoints)
float rollTarget = 0.0;
float pitchTarget = 0.0; 
float yawTarget = 0.0;

unsigned long lastSendTime = 0;
unsigned long lastRecvTime = 0;
const unsigned long interval = 50; 

uint16_t txSeq = 0;
uint16_t lastRxSeq = 0;

//limits output to -45 to 45
float calculatePID(PIDController *pid, float setpoint, float input, float dt) {
  float error = setpoint - input;
  
  // Proportional term
  float pTerm = pid->kP * error;
  
  // Integral term (with windup protection)
  pid->integral += error * dt;
  pid->integral = constrain(pid->integral, -50.0, 50.0);  // Prevent windup
  float iTerm = pid->kI * pid->integral;
  
  // Derivative term
  float dTerm = 0;
  if (dt > 0) {
    dTerm = pid->kD * (error - pid->previousError) / dt;
  }
  
  pid->previousError = error;
  pid->output = pTerm + iTerm + dTerm;
  
  return constrain(pid->output, -45.0, 45.0);  // Limit to servo range
}

float voltageReading() {
  int total = 0;

  // Take multiple samples
  for (int i = 0; i < voltageSamples; i++) {
    total += analogRead(A0);
  }

  // Calculate average ADC value
  float avgADC = total / (float)voltageSamples;

  // Calculate voltage at A0 pin
  float voltageAtA0 = avgADC * (5.0 / 1023.0);

  // Calculate battery voltage using voltage divider formula
  float batteryVoltage = voltageAtA0 * (100000 + 10000) / 10000;

  return batteryVoltage;
}

void failsafe() {
  lastGoodControl.throttle = 0;
  lastGoodControl.aileronL = 35;
  lastGoodControl.aileronR = -35;
  lastGoodControl.elevator = 10;
  lastGoodControl.rudder = 0;
  lastGoodControl.autostabilize = 1;
  

  rollPID.integral = 0;
  rollPID.previousError = 0;
  pitchPID.integral = 0;
  pitchPID.previousError = 0;
  yawPID.integral = 0;
  yawPID.previousError = 0;

  Serial.println("Failsafe: no valid control packet >500ms");
}

void applyControls(const ControlPacket &c) {
  int8_t finalAileronL = c.aileronL;
  int8_t finalAileronR = c.aileronR;
  int8_t finalElevator = c.elevator;
  int8_t finalRudder = c.rudder;
  
  if (c.autostabilize) {
    unsigned long currentTime = millis();
    float dt = (currentTime - rollPID.lastTime) / 1000.0;
    
    if (dt > 0.01) {  // Only update at reasonable intervals
      // Get current angles from Madgwick filter
      float currentRoll = filter.getRoll();
      float currentPitch = filter.getPitch();
      float currentYaw = filter.getYaw();
      
      // Calculate PID corrections
      float rollCorrection = calculatePID(&rollPID, rollTarget, currentRoll, dt);
      float pitchCorrection = calculatePID(&pitchPID, pitchTarget, currentPitch, dt);
      float yawCorrection = calculatePID(&yawPID, yawTarget, currentYaw, dt);
      
      // Mix manual input with stabilization (stab can do up to 15-30 deg auto)
      finalAileronL = c.aileronL - constrain(rollCorrection, -30, 30);
      finalAileronR = c.aileronR + constrain(rollCorrection, -30, 30);
      finalElevator = c.elevator + constrain(pitchCorrection, -15, 15);
      //finalRudder = c.rudder + constrain(yawCorrection, -15, 15);
      
      rollPID.lastTime = currentTime;
      pitchPID.lastTime = currentTime;
      yawPID.lastTime = currentTime;
    }
  }
  
  // Constrain final control values
  finalAileronL = constrain(finalAileronL, -60, 60);
  finalAileronR = constrain(finalAileronR, -60, 60);
  finalElevator = constrain(finalElevator, -30, 30);
  finalRudder = constrain(finalRudder, -30, 30);

  // Apply final control values
  esc.writeMicroseconds(map(c.throttle, 0, 100, 1000, 2000));
  servoAilL.write(90 + finalAileronL);
  servoAilR.write(90 + finalAileronR);
  servoElev.write(90 + finalElevator);
  servoRudd.write(90 + finalRudder);
}

void setup() {
  mcusr_status = MCUSR; // Read the status register
  MCUSR = 0;           // Clear the status register 
  Serial.begin(115200);
  bool wdR = false;

  //may not work due to clearing
  if (mcusr_status & (1 << WDRF)) {
    Serial.println("Watchdog Reset Detected!");
    wdR = true;
  } else {
    Serial.println("Other Reset (e.g., Power-on, External, Brown-out)");
  }

  wdt_enable(WDTO_4S); 

  servoAilL.attach(6);
  servoAilR.attach(7);
  servoElev.attach(4);
  servoRudd.attach(5);
  esc.attach(3);

  //failsafe();

  // Initialize PID timing
  unsigned long currentTime = millis();
  rollPID.lastTime = currentTime;
  pitchPID.lastTime = currentTime;
  yawPID.lastTime = currentTime;

  Wire.begin();
  mpu.initialize();

  //first time (ground) calibration
  if (!wdR) {
    mpu.CalibrateAccel();
    mpu.CalibrateGyro();
  }

  filter.begin(50);
  

  radio.begin();
  radio.setChannel(90);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipeTX);
  radio.openReadingPipe(1, pipeRX);
  radio.startListening();
}

void loop() {
  wdt_reset();
  unsigned long now = millis();

  // Check for control packet
  if (radio.available()) {
    radio.read(&control, sizeof(control));

    uint8_t cs = calculateChecksum((uint8_t*)&control, sizeof(control) - 1);

    if (cs == control.checksum) {
      lastRecvTime = now;
      lastGoodControl = control;
      lastRxSeq = control.seq;
      applyControls(control);
      Serial.print("RX Seq: ");
      Serial.print(control.seq);
      Serial.println(" RX OK");
    } else {
      Serial.println("Bad checksum, ignoring control packet");
      applyControls(lastGoodControl);
    }
  }

  if (now - lastRecvTime > 1000) {
    failsafe();
    // Apply failsafe controls (autostab will be 0 now)
    applyControls(lastGoodControl);
  }

  // Send telemetry at ~20Hz
  if (now - lastSendTime >= interval) {
    lastSendTime = now;

    // Read gyro
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    float lastVoltage = voltageReading();
    telemetry.voltage = lastVoltage;
    telemetry.seq = txSeq++;

    //degree per sec
    float gyroX = gx / 131.0f;
    float gyroY = gy / 131.0f; 
    float gyroZ = gz / 131.0f;

    // g force
    float accelX = ax / 16384.0f;
    float accelY = ay / 16384.0f;
    float accelZ = az / 16384.0f;

    // In loop, before filter.updateIMU():
    static unsigned long lastFilterTime = 0;
    unsigned long currentTime = millis();
    float filterDt = (currentTime - lastFilterTime) / 1000.0;

    if (filterDt >= 0.01) {  // Update filter at max 100Hz
      filter.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);
      lastFilterTime = currentTime;
    }

    telemetry.roll = filter.getRoll();
    telemetry.pitch = filter.getPitch();
    telemetry.yaw = filter.getYaw();

    telemetry.throttleActual = lastGoodControl.throttle;
    telemetry.aileronLActual = lastGoodControl.aileronL;
    telemetry.aileronRActual = lastGoodControl.aileronR;
    telemetry.elevatorActual = lastGoodControl.elevator;
    telemetry.rudderActual = lastGoodControl.rudder;

    telemetry.checksum = calculateChecksum((uint8_t*)&telemetry, sizeof(telemetry) - 1);

    radio.stopListening();
    bool sent = radio.write(&telemetry, sizeof(telemetry));
    radio.startListening();

    if (!sent) Serial.println("Telemetry TX FAIL");
    else Serial.print("TX Seq: "), Serial.print(telemetry.seq), Serial.println(" TX OK");
  }
}
