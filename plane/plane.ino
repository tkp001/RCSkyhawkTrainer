#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "I2Cdev.h"
#include <avr/io.h>
#include <avr/wdt.h>

RF24 radio(9, 10);
const uint64_t pipeTX = 0xF0F0F0F0D2LL; // Plane TX -> Computer RX
const uint64_t pipeRX = 0xF0F0F0F0E1LL; // Plane RX <- Computer TX
const int voltageSamples = 10;
uint8_t mcusr_status;

Servo servoAilL, servoAilR, servoElev, servoRudd, esc;

MPU6050 mpu;
int const interrupt_pin = 2;
bool dmpReady = false;
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;
//VectorInt16 aaReal;
//VectorInt16 aaWorld;
VectorFloat gravity;
VectorInt16 aa;
VectorInt16 aaReal;
float yaw, pitch, roll, aRealX, aRealY, aRealZ;
static float latestYaw, latestPitch, latestRoll, latestRealX, latestRealY, latestRealZ;
float ypr[3];

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
  float lastSetpoint;
  float dTermFiltered;
};

volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

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
  //for setpoint changes
  if (fabs(setpoint - pid->lastSetpoint) > 0.1) {
    pid->integral = 0; // reset I-term
  }
  pid->lastSetpoint = setpoint;

  float error = setpoint - input;
  
  // Proportional term
  float pTerm = pid->kP * error;
  
  // Integral term (with windup protection)
  pid->integral += error * dt;
  pid->integral = constrain(pid->integral, -50.0, 50.0);  // Prevent windup
  float iTerm = pid->kI * pid->integral;
  
  // // Derivative term
  // float dTerm = 0;
  // if (dt > 0) {
  //   dTerm = pid->kD * (error - pid->previousError) / dt;
  // }

  // Add derivative smoothing (simple LPF)
  float rawDTerm = pid->kD * (error - pid->previousError) / dt;
  const float dFilterFactor = 0.1; // 0.0-1.0 (lower = smoother)
  pid->dTermFiltered += dFilterFactor * (rawDTerm - pid->dTermFiltered);

  pid->previousError = error;
  pid->output = pTerm + iTerm + pid->dTermFiltered;
  
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
  lastGoodControl.autostabilize = 0;
  

  rollPID.integral = 0;
  rollPID.previousError = 0;
  pitchPID.integral = 0;
  pitchPID.previousError = 0;
  yawPID.integral = 0;
  yawPID.previousError = 0;

  //Serial.println("Failsafe: no valid control packet");
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
      float currentRoll = telemetry.roll;
      float currentPitch = telemetry.pitch;
      float currentYaw = telemetry.yaw;

      
      // Calculate PID corrections
      float rollCorrection = calculatePID(&rollPID, rollTarget, currentRoll, dt);
      float pitchCorrection = calculatePID(&pitchPID, pitchTarget, currentPitch, dt);
      float yawCorrection = calculatePID(&yawPID, yawTarget, currentYaw, dt);
      
      // Mix manual input with stabilization (stab can do up to 15-30 deg auto)
      finalAileronL = c.aileronL + constrain(rollCorrection, -30, 30);
      finalAileronR = c.aileronR - constrain(rollCorrection, -30, 30);
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

  //TO ADD: WRITE TO special telemetry variables FOR AUTOSTAB VALUES (true servo value not sent) IF APPLIED

  // Apply final control values
  esc.writeMicroseconds(map(c.throttle, 0, 100, 1000, 2000));
  servoAilL.write(90 + finalAileronL);
  servoAilR.write(90 + finalAileronR);
  servoElev.write(90 + finalElevator);
  servoRudd.write(90 + finalRudder);
}

bool readMPU(float &yaw, float &pitch, float &roll, float &aRealX, float &aRealY, float &aRealZ) {
  wdt_reset();
  if (!dmpReady || !mpuInterrupt) return false; // No new data yet
  mpuInterrupt = false;

  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  // Overflow check
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    return false;
  }

  if (fifoCount < packetSize) return false; // Not enough data yet

  // Read packet
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

  yaw   = ypr[0] * 180 / M_PI;
  pitch = ypr[1] * 180 / M_PI;
  roll  = ypr[2] * 180 / M_PI;
  aRealX = aaReal.x / 16384.0f;
  aRealY = aaReal.y / 16384.0f;
  aRealZ = aaReal.z / 16384.0f;
  
  return true;
}

// //Pull arduino status
// uint8_t mcusr_mirror __attribute__ ((section (".noinit")));

// void getMCUSR(void) \
// __attribute__((naked)) \
// __attribute__((section(".init3")));

// void getMCUSR(void) {
//     mcusr_mirror = MCUSR; // copy before Arduino core clears it
//     MCUSR = 0;            // clear
//     wdt_disable();        // also disable watchdog so it doesn’t loop-reset
// }

void setup() {
  wdt_disable();
  mcusr_status = MCUSR; // Read the status register
  MCUSR = 0; // Clear the status register

  Wire.begin();
  Wire.setClock(400000); // Fast I2C 
  Wire.setWireTimeout(25000, true);
  Serial.begin(115200);
  bool wdR = false;

  // uint8_t mcusr_status = MCUSR;
  // MCUSR = 0;  // clear so next reset cause is fresh

  // if (mcusr_mirror & (1 << PORF)) {
  //   Serial.println("Power-on Reset");
  // }
  // if (mcusr_mirror & (1 << EXTRF)) {
  //   Serial.println("External Reset");
  // }
  // if (mcusr_mirror & (1 << BORF)) {
  //   Serial.println("Brown-out Reset");
  // }
  // if (mcusr_mirror & (1 << WDRF)) {
  //   Serial.println("Watchdog Reset");
  //   wdR = true;
  // }

  //ATTACH SURFACES AND MOTOR
  servoAilL.attach(6);
  servoAilR.attach(7);
  servoElev.attach(4);
  servoRudd.attach(5);
  esc.attach(15); //was D3 before mpu6050

  //failsafe();

  // Initialize PID timing
  unsigned long currentTime = millis();
  rollPID.lastTime = currentTime;
  pitchPID.lastTime = currentTime;
  yawPID.lastTime = currentTime;
  
  //MPU6050 Init All
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(interrupt_pin, INPUT);

  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    //while(true); //for some reason always fails
  } else {
    Serial.println("MPU6050 connection successful");
  }

  // Load and configure DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  //wdt_reset();

  if (devStatus == 0) {   
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    //wdt_reset();
    
      
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(interrupt_pin));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), dmpDataReady, RISING);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println(F("DMP ready!"));
   
    //extra
    mpuIntStatus = mpu.getIntStatus();
    
  } else {
    Serial.print("DMP Init failed (code ");
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }

  //NRF24L01 SETUP
  //wdt_reset();
  radio.begin();
  radio.setChannel(90);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipeTX);
  radio.openReadingPipe(1, pipeRX);
  radio.startListening();

  wdt_enable(WDTO_4S);
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

  //failsafe
  if (now - lastRecvTime > 1000) {
    failsafe();
    // Apply failsafe controls
    applyControls(lastGoodControl);
  }

  // Quick gyro calibration if autostabilize == 2
  if (lastGoodControl.autostabilize == 2) {
    Serial.println("Quick gyro calibration...");
    mpu.CalibrateGyro(6); // quick only
    mpu.CalibrateAccel(6);
  }

  //get MPU6050 data
  if (readMPU(yaw, pitch, roll, aRealX, aRealY, aRealZ)) {
    latestYaw = yaw;
    latestPitch = pitch;
    latestRoll = roll;
    latestRealX = aRealX;
    latestRealY = aRealY;
    latestRealZ = aRealZ;
  } else {
    Wire.clearWireTimeoutFlag();
  }

  if (Wire.getWireTimeoutFlag()) {
    Serial.print("MPU TIMEOUT");
    mpu.initialize();
  }

  // Send telemetry at ~20Hz
  if (now - lastSendTime >= interval) {
    lastSendTime = now;
    float lastVoltage = voltageReading();
    telemetry.voltage = lastVoltage;
    telemetry.seq = txSeq++;

    telemetry.roll = latestRoll;
    telemetry.pitch = latestPitch;
    telemetry.yaw = latestYaw;

    // Serial.print("A1: ");
    // Serial.print(latestRealX);
    // Serial.print(" A2: ");
    // Serial.print(latestRealY);
    // Serial.print(" A3: ");
    // Serial.print(latestRealZ);
    // Serial.println(" | ");


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
