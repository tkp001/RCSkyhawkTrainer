#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
//#include <ServoTimer2.h>
#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>
#include "I2Cdev.h"
#include <avr/io.h>
#include <avr/wdt.h>

RF24 radio(9, 10);
const uint64_t pipeTX = 0xF0F0F0F0D2LL; // Plane TX -> Computer RX
const uint64_t pipeRX = 0xF0F0F0F0E1LL; // Plane RX <- Computer TX
const int voltageSamples = 10;
uint8_t mcusr_status;
bool failsafeActivated;

Servo servoAilL, servoAilR, servoElev, servoRudd;
Servo esc;

MPU6050 mpu;    
// uint16_t packetSize;    
// uint16_t fifoCount;     
// uint8_t fifoBuffer[64]; 

Madgwick filter; // Madgwick filter instance
unsigned long lastFilterUpdate = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
bool gyroCalibrated = false;

// Quaternion q;
//VectorInt16 aaReal;
//VectorInt16 aaWorld;
// VectorFloat gravity;
// VectorInt16 aa;
// VectorInt16 aaReal;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float yaw, pitch, roll, accelX, accelY, accelZ;
static float latestYaw, latestPitch, latestRoll, latestAccelX, latestAccelY, latestAccelZ;
int8_t finalAileronL, finalAileronR, finalElevator, finalRudder;
// float ypr[3];

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

uint8_t calculateChecksum(uint8_t *data, size_t length) {
  uint8_t sum = 0;
  for (size_t i = 0; i < length; i++) sum ^= data[i];
  return sum;
}

int atoMicro(int angle) {
  return map(angle, 0, 180, 1000, 2000);
}

void clearI2CBus() {
  pinMode(SCL, OUTPUT);
  for (int i=0; i<9; i++) {
    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
  }
  pinMode(SCL, INPUT);
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
  lastGoodControl.aileronL = 0;
  lastGoodControl.aileronR = 0;
  lastGoodControl.elevator = 0;
  lastGoodControl.rudder = 0;
  lastGoodControl.autostabilize = 4; //1 before

  // rollPID.integral = 0;
  // rollPID.previousError = 0;
  // pitchPID.integral = 0;
  // pitchPID.previousError = 0;
  // yawPID.integral = 0;
  // yawPID.previousError = 0;

  //Serial.println("Failsafe: no valid control packet");
}

void applyControls(const ControlPacket &c) {
  //Variables made public
  finalAileronL = c.aileronL;
  finalAileronR = c.aileronR;
  finalElevator = c.elevator;
  finalRudder = c.rudder;
  
  //Quick Actions
  // Manual calibration if on ground (throttle 0)
  if (c.autostabilize == 2 && c.throttle == 0) {
    // Serial.println("Quick gyro calibration...");
    esc.detach();
    manualCalibrateGyro();
    esc.attach(15);
  }

  // Hold to auto level if autostabilize == 3
  if (c.autostabilize == 3) {
    // Serial.println("Leveling.");
    rollTarget = 0.0;
    pitchTarget = 0.0;
    yawTarget = 0.0;
  }

  if (c.autostabilize > 0) {
    unsigned long currentTime = millis();
    float dt = (currentTime - rollPID.lastTime) / 1000.0;
    
    if (dt > 0.01) {  // Only update at reasonable intervals
      float currentRoll = latestRoll; //telemetry.roll
      float currentPitch = latestPitch;
      float currentYaw = latestYaw;
      static unsigned long failsafeStartTime = 0;
      static bool failsafePIDReset = false;

      if (!failsafeActivated && c.autostabilize != 4) {
        failsafePIDReset = false;  // Reset flag when exiting failsafe
        failsafeStartTime = 0;
      }

      if (c.autostabilize == 4 || failsafeActivated) {  // Failsafe mode
        // static unsigned long failsafeStartTime = 0;
        // static bool failsafePIDReset = false;
        
        if (!failsafePIDReset) {
            // Reset PIDs once at failsafe start
            rollPID.integral = 0; rollPID.previousError = 0;
            pitchPID.integral = 0; pitchPID.previousError = 0;
            yawPID.integral = 0; yawPID.previousError = 0;
            failsafeStartTime = millis();
            failsafePIDReset = true;
        }
        
        unsigned long failsafeTime = millis() - failsafeStartTime;
        if (failsafeTime < 3000) {
            rollTarget = 0.0;    // Level
            pitchTarget = -5.0;  // Nose down
        } else {
            rollTarget = 20.0;   // Spiral
            pitchTarget = -2.0;
        }
        yawTarget = 0.0;

      } else if (c.autostabilize == 1) {
          // Static variables to remember targets and hold states
          static float savedRollTarget = 0;
          static float savedPitchTarget = 0; 
          static float savedYawTarget = 0;
          static bool rollHolding = false;
          static bool pitchHolding = false;
          static bool yawHolding = false;
          
          // ROLL HOLD - Fix: use average of both ailerons
          float rollInput = (c.aileronL + c.aileronR) / 2.0;
          if (abs(rollInput) < 5) {  // Ailerons centered
              if (!rollHolding) {
                  savedRollTarget = currentRoll;  // Save current angle once
                  rollHolding = true;
              }
              rollTarget = savedRollTarget;  // Use saved target
          } else {
              rollHolding = false;  // Exit hold mode when stick moves
          }
          
          // PITCH HOLD
          if (abs(c.elevator) < 5) {  // Elevator centered
              if (!pitchHolding) {
                  savedPitchTarget = currentPitch;  // Save current angle once
                  pitchHolding = true;
              }
              pitchTarget = savedPitchTarget;  // Use saved target
          } else {
              pitchHolding = false;  // Exit hold mode when stick moves
          }
          

          // YAW HOLD
          if (abs(c.rudder) < 5) {  // Rudder centered
              if (!yawHolding) {
                  savedYawTarget = currentYaw;    // Save current angle once
                  yawHolding = true;
              }
              yawTarget = savedYawTarget;  // Use saved target
          } else {
              yawHolding = false;  // Exit hold mode when stick moves
          }
      }

      // Calculate PID corrections
      float rollCorrection = calculatePID(&rollPID, rollTarget, currentRoll, dt);
      float pitchCorrection = calculatePID(&pitchPID, pitchTarget, currentPitch, dt);
      float yawCorrection = calculatePID(&yawPID, yawTarget, currentYaw, dt);
      
      // Mix manual input with stabilization (stab can do up to 15-30 deg auto)
      //Aileron mapping is correct, it has worked through testing
      finalAileronL = c.aileronL - constrain(rollCorrection, -45, 45);
      finalAileronR = c.aileronR - constrain(rollCorrection, -45, 45); // changed
      finalElevator = c.elevator - constrain(pitchCorrection, -30, 30);
      //finalRudder = c.rudder + constrain(yawCorrection, -15, 15);
      
      rollPID.lastTime = currentTime;
      pitchPID.lastTime = currentTime;
      yawPID.lastTime = currentTime;
    }
  }
  
  // Constrain final control values
  finalAileronL = constrain(finalAileronL, -60, 60);
  finalAileronR = constrain(finalAileronR, -60, 60);
  finalElevator = constrain(finalElevator, -45, 45);
  finalRudder = constrain(finalRudder, -45, 45);

  // Apply final control values
  esc.writeMicroseconds(map(c.throttle, 0, 100, 1000, 2000));
  servoAilL.write(90 + finalAileronL);
  servoAilR.write(90 + finalAileronR);
  servoElev.write(90 + finalElevator);
  servoRudd.write(90 + finalRudder);
}

bool readMPU(float &yaw, float &pitch, float &roll, float &accelX, float &accelY, float &accelZ) {
  unsigned long currentTime = millis();
  
  // Calculate delta time in seconds
  float dt = (currentTime - lastFilterUpdate) / 1000.0f;
  if (dt <= 0) dt = 0.01f; // Prevent divide by zero
  lastFilterUpdate = currentTime;

  // Read raw sensor data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert accelerometer readings to g-force (±2g range)
  accelX = ax / 16384.0f;
  accelY = ay / 16384.0f;  
  accelZ = az / 16384.0f;
  
  // Convert gyroscope readings to degrees/second (±250°/s range) and apply calibration
  float gyroX = (gx / 131.0f) - gyroOffsetX;
  float gyroY = (gy / 131.0f) - gyroOffsetY;
  float gyroZ = (gz / 131.0f) - gyroOffsetZ;
  
  // Update Madgwick filter (library expects degrees/second for gyro)
  filter.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);
  
  // Get Euler angles from filter
  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();
  
  return true;  // Always successful with raw readings
}

// ADD new calibration function to replace DMP calibration
void manualCalibrateGyro() {
    Serial.println("Calibrating gyroscope...");
    
    const int numSamples = 1000;
    long gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
    
    for (int i = 0; i < numSamples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        gyroXSum += gx;
        gyroYSum += gy;
        gyroZSum += gz;
        
        delay(1);
        
        // Show progress every 100 samples
        if (i % 100 == 0) {
            Serial.print(".");
        }
    }
    Serial.println();
    
    // Calculate offsets in degrees/second
    gyroOffsetX = (gyroXSum / numSamples) / 131.0f;
    gyroOffsetY = (gyroYSum / numSamples) / 131.0f;
    gyroOffsetZ = (gyroZSum / numSamples) / 131.0f;
    
    gyroCalibrated = true;
    
    Serial.print("Gyro offsets: X=");
    Serial.print(gyroOffsetX);
    Serial.print(", Y=");
    Serial.print(gyroOffsetY);
    Serial.print(", Z=");
    Serial.println(gyroOffsetZ);
    Serial.println("Gyro calibration complete!");
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
  Wire.setClock(100000); // I2C 
  // Wire.setWireTimeout(25000, true);
  // Wire.clearWireTimeoutFlag();
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

  //failsafe();

  // Initialize PID timing
  unsigned long currentTime = millis();
  rollPID.lastTime = currentTime;
  pitchPID.lastTime = currentTime;
  yawPID.lastTime = currentTime;
  
  //MPU6050 Init All
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // ±250°/s
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // ±2g
  mpu.setDLPFMode(MPU6050_DLPF_BW_42); // 42Hz cutoff
  // mpu.reset();
  // mpu.initialize();

  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    //while(true); //for some reason always fails
  } else {
    Serial.println("MPU6050 connection successful");
  }
  filter.begin(50); // 50Hz update rate (adjust based on your loop frequency)
  
  lastFilterUpdate = millis();
  Serial.println("MPU6050 and Madgwick filter ready!");

  Serial.println("Updating internal sensor offsets...\n");
  manualCalibrateGyro();

  //MPU6050_Zero

  //DMP removed

  //NRF24L01 SETUP
  radio.begin();

  if (!radio.isChipConnected()) {
    Serial.println("NRF24L01+ not detected!");
  } else {
    Serial.println("NRF24L01+ detected");
  }

  radio.setChannel(90);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openWritingPipe(pipeTX);
  radio.openReadingPipe(1, pipeRX);
  radio.startListening();

  //ATTACH SURFACES AND MOTOR
  failsafe();
  servoAilL.attach(6);
  servoAilR.attach(7);
  servoElev.attach(4);
  servoRudd.attach(5);
  esc.attach(15);
  applyControls(lastGoodControl);

  wdt_enable(WDTO_2S);
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
      // Serial.print("RX Seq: ");
      // Serial.print(control.seq);
      // Serial.println(" RX OK");
    } else {
      // Serial.println("Bad checksum, ignoring control packet");
      applyControls(lastGoodControl);
    }
  }

  //failsafe
  if (now - lastRecvTime > 1000) {
    failsafeActivated = true;
    failsafe();
    // Apply failsafe controls
    applyControls(lastGoodControl);
  } else {
    failsafeActivated = false;
  }

  //get MPU6050 data
  if (readMPU(yaw, pitch, roll, accelX, accelY, accelZ)) {
    latestYaw = yaw;
    latestPitch = pitch;
    latestRoll = roll;
    latestAccelX = accelX;
    latestAccelY = accelY;
    latestAccelZ = accelZ;
  } else {
    //Wire.clearWireTimeoutFlag();
    Serial.println("Failed data read somehow.");
  }

  //Wire timeout gone

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


    // telemetry.throttleActual = lastGoodControl.throttle;
    // telemetry.aileronLActual = lastGoodControl.aileronL;
    // telemetry.aileronRActual = lastGoodControl.aileronR;
    // telemetry.elevatorActual = lastGoodControl.elevator;
    // telemetry.rudderActual = lastGoodControl.rudder;

    //Actual values applied to servos directly.
    telemetry.throttleActual = lastGoodControl.throttle;
    telemetry.aileronLActual = finalAileronL;  // Send actual servo positions
    telemetry.aileronRActual = finalAileronR;
    telemetry.elevatorActual = finalElevator;
    telemetry.rudderActual = finalRudder;

    telemetry.checksum = calculateChecksum((uint8_t*)&telemetry, sizeof(telemetry) - 1);

    radio.stopListening();
    bool sent = radio.write(&telemetry, sizeof(telemetry));
    radio.startListening();

    // if (!sent) Serial.println("Telemetry TX FAIL");
    // else Serial.print("TX Seq: "), Serial.print(telemetry.seq), Serial.println(" TX OK");
  }
}