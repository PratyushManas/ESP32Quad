/*
 * ============================================================
 *  ESP32 Flight Controller v4  (+ telemetry)
 * ============================================================
 *  Phone hosts a hotspot.
 *  ESP32 connects to that hotspot as a WiFi client.
 *  Python app on phone sends UDP packets to ESP32 (port 5005).
 *  ESP32 sends text telemetry back (port 5006) - readable by
 *  the phone app's message console AND/OR the PC logger script
 *  at the same time, since they're on separate ports.
 *
 *  Control packet IN (16 bytes, little-endian floats):
 *    float roll, pitch, yaw, thrust
 *
 *  Telemetry OUT (plain text):
 *    On connect       -> "Drone Connected"
 *    2s later, then
 *    every 200ms       -> "THR:55% R:-2.1 P:0.4 Y:12.0
 *                          GX:0.10 GY:-0.05 GZ:1.2
 *                          AX:0.02 AY:-0.01 AZ:9.79"
 *
 *  Motor Layout (top-down):
 *           FRONT
 *    M1 (CW)     M2 (CCW)
 *       \           /
 *        [  ESP32  ]
 *       /           \
 *    M4 (CCW)    M3 (CW)
 *           BACK
 *
 *  Wiring:
 *    MPU6050 SDA -> GPIO 21
 *    MPU6050 SCL -> GPIO 22
 *    M1 ESC      -> GPIO 15
 *    M2 ESC      -> GPIO 16
 *    M3 ESC      -> GPIO 18
 *    M4 ESC      -> GPIO 19
 *
 *  Libraries:
 *    - ESP32Servo
 *    - Adafruit MPU6050
 *    - Adafruit Unified Sensor
 *    - Adafruit BusIO
 * ============================================================
 */

#include <ESP32Servo.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Math.h>

// ============================================================
//  *** CHANGE THESE to match your phone hotspot ***
// ============================================================
const char* HOTSPOT_SSID     = "Manas";
const char* HOTSPOT_PASSWORD = "jairajputana";

// Control packets arrive here — must match the Python app
const int UDP_PORT = 5005;

// Telemetry text goes out here — separate port so the phone app
// and a PC logger can both listen at the same time without conflict
const int TELEMETRY_PORT = 5006;

// ============================================================
//  ESC / Motor pins
// ============================================================
#define M1_PIN  15
#define M2_PIN  16
#define M3_PIN  18
#define M4_PIN  19
#define ESC_MIN 1000
#define ESC_MAX 2000

Servo m1, m2, m3, m4;

// ============================================================
//  WiFi / UDP
// ============================================================
WiFiUDP udp;            // control packets in
WiFiUDP udpTelemetry;   // telemetry text out

IPAddress phoneIP;
bool      phoneIPKnown = false;

// ============================================================
//  IMU
// ============================================================
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;

float gyroOffX, gyroOffY, gyroOffZ;
float gyroX, gyroY, gyroZ;   // deg/s
float accX,  accY,  accZ;    // m/s^2
float rollAcc = 0.0;
float pitchAcc = 0.0;


// ============================================================
//  Control inputs
// ============================================================
float cmdRoll    = 0;
float cmdPitch   = 0;
float cmdYaw     = 0;
float cmdThrust  = 0;

uint32_t previousTime = 0;
float dt = 0.0;

const float alpha = 0.98;

String input;

//variables for PID control
float currentRoll, currentPitch, currentYaw;

float errRoll, errPitch, errYaw;
float errSumRoll = 0, errSumPitch = 0, errSumYaw = 0;
float prevErrRoll = 0, prevErrPitch = 0, prevErrYaw = 0;

//PID constants [the actual part that requires tuning]
float KpR = 1.2, KiR = 0.0, KdR = 0.005;
float KpP = 1.2, KiP = 0.0, KdP = 0.005;
float KpY = 1.5, KiY = 0.0, KdY = 0.0;

bool     connected  = false;
uint32_t lastPktMs  = 0;
const uint32_t TIMEOUT_MS = 1000;

// ============================================================
//  Telemetry timing
// ============================================================
bool     connectedMsgSent       = false;  // "Drone Connected" sent once
uint32_t connectedAtMs          = 0;      // when connection was confirmed
bool     telemetryStarted       = false;  // 2s delay elapsed
uint32_t lastTelemetryMs        = 0;
const uint32_t TELEMETRY_INTERVAL_MS    = 200;   // 5 Hz telemetry rate
const uint32_t TELEMETRY_START_DELAY_MS = 2000;  // 2s after connect

// ============================================================
//  Forward declarations
// ============================================================
void allMotors(int us);
void readIMU();
void readUDP();
void sendTelemetryText(const String& msg);
void handleTelemetryTiming();
void computePID();
void calculateError();
void processCommand(String cmd);
void printPIDValues();
void updateAttitude();

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Flight Controller v4 (+telemetry) ===");

  // ── 1. Arm ESCs ────────────────────────────────────────
            ESP32PWM::allocateTimer(0);
            ESP32PWM::allocateTimer(1);
            ESP32PWM::allocateTimer(2);
            ESP32PWM::allocateTimer(3);

            m1.setPeriodHertz(50); m2.setPeriodHertz(50);
            m3.setPeriodHertz(50); m4.setPeriodHertz(50);

            m1.attach(M1_PIN, ESC_MIN, ESC_MAX);
            m2.attach(M2_PIN, ESC_MIN, ESC_MAX);
            m3.attach(M3_PIN, ESC_MIN, ESC_MAX);
            m4.attach(M4_PIN, ESC_MIN, ESC_MAX);

            Serial.println("[ESC] Arming...");
            allMotors(2000); delay(3000);
            allMotors(ESC_MIN); delay(2000);
            Serial.println("[ESC] Armed.");

  // ── 2. IMU setup ───────────────────────────────────────
            Wire.begin(21, 22);
            if (!mpu.begin()) {
              Serial.println("[IMU] MPU6050 not found!");
              while (true) { allMotors(ESC_MIN); delay(100); }
            }
            mpu.setGyroRange(MPU6050_RANGE_500_DEG);
            mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
            mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
            Serial.println("[IMU] Ready.");

            // ── 3. Gyro calibration ────────────────────────────────
            Serial.println("[IMU] Calibrating - keep still...");
            float sx = 0, sy = 0, sz = 0;
            for (int i = 0; i < 500; i++) {
              sensors_event_t a, g, t;
              mpu.getEvent(&a, &g, &t);
              sx += g.gyro.x; sy += g.gyro.y; sz += g.gyro.z;
              if (i % 50 == 0) allMotors(ESC_MIN);
            }
            gyroOffX = sx / 500;
            gyroOffY = sy / 500;
            gyroOffZ = sz / 500;
            Serial.printf("[IMU] Offsets: X=%.4f Y=%.4f Z=%.4f\n",
                          gyroOffX, gyroOffY, gyroOffZ);

       
        //  Complementary filter initialization
        previousTime = micros();

        mpu.getEvent(&a, &g, &temp);

        accX = a.acceleration.x;
        accY = a.acceleration.y;
        accZ = a.acceleration.z;

        currentRoll =
            atan2(accY, accZ) * RAD_TO_DEG;

        currentPitch =
            atan2(-accX,
                  sqrt(accY * accY + accZ * accZ))
            * RAD_TO_DEG;

        currentYaw = 0;


  // ── 4. Connect to phone hotspot ────────────────────────
            Serial.printf("[WiFi] Connecting to '%s'...\n", HOTSPOT_SSID);
            WiFi.mode(WIFI_STA);
            WiFi.begin(HOTSPOT_SSID, HOTSPOT_PASSWORD);

            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 30) {
              allMotors(ESC_MIN);   // keep ESCs alive during connect
              delay(500);
              Serial.print(".");
              attempts++;
            }

            if (WiFi.status() == WL_CONNECTED) {
              Serial.printf("\n[WiFi] Connected! ESP32 IP: %s\n",
                            WiFi.localIP().toString().c_str());
              udp.begin(UDP_PORT);
              udpTelemetry.begin(TELEMETRY_PORT);
              Serial.printf("[UDP] Control on :%d  Telemetry on :%d\n",
                            UDP_PORT, TELEMETRY_PORT);
            } else {
              Serial.println("\n[WiFi] FAILED to connect. Check SSID/password.");
              // Keep running so motors stay at ESC_MIN
            }

            Serial.println("[FC] Ready. Enter ESP32 IP in app and tap Connect.");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {

  // 1. Read UDP (also detects new connection + remembers phone IP)
       readUDP();

  // 2. Timeout — cut motors if no packet, reset telemetry state
        if (millis() - lastPktMs > TIMEOUT_MS) {
          if (connected) {
            // Link just dropped — reset so a reconnect re-announces itself
            connected        = false;
            connectedMsgSent = false;
            telemetryStarted = false;
            phoneIPKnown      = false;
          }
          cmdThrust = 0;
        }

  // 3. Read IMU
        readIMU();

  // 4. Handle "Drone Connected" message + 2s delayed telemetry start
        handleTelemetryTiming();

  // 5. Motors off if not connected or no throttle
        if (!connected || cmdThrust < 0.05f) {
          allMotors(ESC_MIN);
          return;
        }
          
        updateAttitude();

  //calculating error for PID computation - IMU values are already read in this loop so no need to call it again
        calculateError();

  //calling the PID computation and writing the motor values int the function itself, only if the above if() clause is false.
        computePID();


  //Updating PID coefficients
        if (Serial.available())
          {
              input = Serial.readStringUntil('\n');
              input.trim();        // remove \r and spaces

              if (input == "PID")
              {
                printPIDValues();
              }
              else
              {
                processCommand(input);
              }
          }


  // 9. Debug every 500ms (Serial only)
        static uint32_t lastDbg = 0;
        if (millis() - lastDbg > 500) {
          lastDbg = millis();
          Serial.printf("[FC] T:%.2f R:%.1f P:%.1f Y:%.1f",
                        cmdThrust, cmdRoll, cmdPitch, cmdYaw);
        }
}


// ============================================================
//  Read UDP packet from Python app + detect connection
// ============================================================
void readUDP() {
  int sz = udp.parsePacket();
  if (sz < 16) return;   // expect exactly 16 bytes (4 floats)

  uint8_t buf[16];
  udp.read(buf, 16);

  // Unpack 4 little-endian floats
  memcpy(&cmdRoll,   buf,      4);
  memcpy(&cmdPitch,  buf +  4, 4);
  memcpy(&cmdYaw,    buf +  8, 4);
  memcpy(&cmdThrust, buf + 12, 4);

  cmdThrust = (cmdThrust) * 2.0f;   // remap 0.5 centre -> 0.0

  // Clamp for safety
  cmdRoll   = constrain(cmdRoll,   -30.0f,  30.0f);
  cmdPitch  = constrain(cmdPitch,  -30.0f,  30.0f);
  cmdYaw    = constrain(cmdYaw,   -100.0f, 100.0f);
  cmdThrust = constrain(cmdThrust,  0.0f,    1.0f);

  // First packet of this session -> remember sender's IP for
  // telemetry replies, and mark as newly connected
  if (!connected) {
    phoneIP      = udp.remoteIP();
    phoneIPKnown = true;
    Serial.printf("[UDP] Phone connected from %s\n", phoneIP.toString().c_str());
  }

  connected  = true;
  lastPktMs  = millis();
}


// ============================================================
//  Telemetry timing — "Drone Connected" then delayed data feed
// ============================================================
void handleTelemetryTiming() {
  if (!connected || !phoneIPKnown) return;

  // Send the one-time connection confirmation
  if (!connectedMsgSent) {
    sendTelemetryText("Drone Connected");
    connectedMsgSent = true;
    connectedAtMs    = millis();
    Serial.println("[TELEM] Sent: Drone Connected");
    return;
  }

  // Wait 2 seconds after connection before starting data telemetry
  if (!telemetryStarted) {
    if (millis() - connectedAtMs >= TELEMETRY_START_DELAY_MS) {
      telemetryStarted = true;
      Serial.println("[TELEM] Starting periodic telemetry.");
    }
    return;
  }

  // Send throttle % + IMU data at a fixed interval
  if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = millis();

    int throttlePct = (int)(cmdThrust * 100.0f);

    char msg[160];
    snprintf(msg, sizeof(msg),
             "THR:%d%% R:%.1f P:%.1f Y:%.1f GX:%.2f GY:%.2f GZ:%.2f AX:%.2f AY:%.2f AZ:%.2f",
             throttlePct, cmdRoll, cmdPitch, cmdYaw,
             gyroX, gyroY, gyroZ, accX, accY, accZ);

    sendTelemetryText(String(msg));
  }
}


// ============================================================
//  Send a plain text telemetry message to the phone
// ============================================================
void sendTelemetryText(const String& msg) {
  if (!phoneIPKnown) return;
  udpTelemetry.beginPacket(phoneIP, TELEMETRY_PORT);
  udpTelemetry.write((const uint8_t*)msg.c_str(), msg.length());
  udpTelemetry.endPacket();
}

//  Attitude calculation
void updateAttitude()
{
    //-----------------------------------
    // Time
    //-----------------------------------

    uint32_t now = micros();

    dt = (now - previousTime) * 1e-6f;

    previousTime = now;

    if(dt <= 0 || dt > 0.05f)
        return;


    //-----------------------------------
    // Accelerometer angles
    //-----------------------------------

    rollAcc =
        atan2(accY, accZ) * RAD_TO_DEG;

    pitchAcc =
        atan2(-accX,
              sqrt(accY * accY + accZ * accZ))
        * RAD_TO_DEG;

    //-----------------------------------
    // Integrate gyro
    //-----------------------------------

    currentRoll += gyroX * dt;
    currentPitch += gyroY * dt;
    currentYaw += gyroZ * dt;

    //-----------------------------------
    // Complementary Filter
    //-----------------------------------

    currentRoll =
        alpha * currentRoll +
        (1.0f - alpha) * rollAcc;

    currentPitch =
        alpha * currentPitch +
        (1.0f - alpha) * pitchAcc;
}


//Error Calculation for PID loop
void calculateError() {
  //calculating current Roll, Pitch, Yaw in terms of angles
    errRoll  = cmdRoll  - currentRoll;
    errPitch = cmdPitch - currentPitch;

    // Rate mode for yaw
    errYaw = cmdYaw - gyroZ;
}


//Actual PID computation
void computePID() {
    float deltaErrRoll = 0, deltaErrPitch = 0, deltaErrYaw = 0;          // Error deltas in that order   : Yaw, Pitch, Roll
    float yaw_pid      = 0;
    float pitch_pid    = 0;
    float roll_pid     = 0;

      // Calculate sum of errors : Integral coefficients
      errSumYaw     +=  errYaw    * dt;
      errSumPitch   +=  errPitch  * dt;
      errSumRoll    +=  errRoll   * dt;

      // Clamping the error sums
      errSumYaw   = constrain(errSumYaw, -100, 100);
      errSumPitch = constrain(errSumPitch, -100, 100);
      errSumRoll  = constrain(errSumRoll, -100, 100);
      
      // Calculate error delta : Derivative coefficients
      deltaErrYaw     = (errYaw    - prevErrYaw)    / (dt);
      deltaErrPitch   = (errPitch  - prevErrPitch)  / (dt);
      deltaErrRoll    = (errRoll   - prevErrRoll)   / (dt);
      
      // Save current error as previous_error for next time
      prevErrRoll = errRoll;
      prevErrPitch = errPitch;
      prevErrYaw = errYaw;

      // PID = e.Kp + ∫e.Ki + Δe.Kd
      yaw_pid     = (errYaw   * KpY)  + (errSumYaw    * KiY)  + (deltaErrYaw    * KdY);
      pitch_pid   = (errPitch * KpP)  + (errSumPitch  * KiP)  + (deltaErrPitch  * KdP);
      roll_pid    = (errRoll  * KpR)  + (errSumRoll   * KiR)  + (deltaErrRoll   * KdR);


      // Cauculate new target throttle for each motor
      // NOTE: These depend on setup of drone. Verify setup is propper and
      //       consider changing the plus and minuses here if issues happen.
      //       If drone is in propper setup these make sense.

      m1.writeMicroseconds(constrain((1000 + (cmdThrust * 1000) - (int)(pitch_pid * 2)  + (int)(roll_pid * 2) - (int)(yaw_pid/2)), ESC_MIN, ESC_MAX));
      m2.writeMicroseconds(constrain((1000 + (cmdThrust * 1000) - (int)(pitch_pid * 2)  - (int)(roll_pid * 2) + (int)(yaw_pid/2)), ESC_MIN, ESC_MAX));
      m3.writeMicroseconds(constrain((1000 + (cmdThrust * 1000) + (int)(pitch_pid * 2)  - (int)(roll_pid * 2) - (int)(yaw_pid/2)), ESC_MIN, ESC_MAX));
      m4.writeMicroseconds(constrain((1000 + (cmdThrust * 1000) + (int)(pitch_pid * 2)  + (int)(roll_pid * 2) + (int)(yaw_pid/2)), ESC_MIN, ESC_MAX));

      // Serial.print("ROLL=");
      // Serial.print(roll_pid);

      // Serial.print(" PITCH=");
      // Serial.print(pitch_pid);

      // Serial.print(" YAW=");
      // Serial.println(yaw_pid);

      //Modification for serial plotter
      Serial.print(roll_pid);
      Serial.print('\t');
      Serial.print(pitch_pid);
      Serial.print('\t');
      Serial.println(yaw_pid);
}


//Updating the PID valued
void processCommand(String cmd)
    {
        int eq = cmd.indexOf('=');

        if (eq == -1)
            return;      // invalid command

        String name = cmd.substring(0, eq);
        float value = cmd.substring(eq + 1).toFloat();

        if (name == "KpR")
            KpR = value;

        else if (name == "KiR")
            KiR = value;

        else if (name == "KdR")
            KdR = value;

        else if (name == "KpP")
            KpP = value;

        else if (name == "KiP")
            KiP = value;

        else if (name == "KdP")
            KdP = value;

        else if (name == "KpY")
            KpY = value;

        else if (name == "KiY")
            KiY = value;

        else if (name == "KdY")
            KdY = value;

        else
            Serial.println("Unknown variable");
    }


//Printing PID values on demand
void printPIDValues()
{
    Serial.println();
    Serial.println("===== Current PID Coefficients =====");

    Serial.print("Roll  : ");
    Serial.print("KpR = "); Serial.print(KpR, 6);
    Serial.print("   KiR = "); Serial.print(KiR, 6);
    Serial.print("   KdR = "); Serial.println(KdR, 6);

    Serial.print("Pitch : ");
    Serial.print("KpP = "); Serial.print(KpP, 6);
    Serial.print("   KiP = "); Serial.print(KiP, 6);
    Serial.print("   KdP = "); Serial.println(KdP, 6);

    Serial.print("Yaw   : ");
    Serial.print("KpY = "); Serial.print(KpY, 6);
    Serial.print("   KiY = "); Serial.print(KiY, 6);
    Serial.print("   KdY = "); Serial.println(KdY, 6);

    Serial.println("====================================");
    Serial.println();
}


// ============================================================
//  Read IMU
// ============================================================
void readIMU() {
    mpu.getEvent(&a, &g, &temp);

    gyroX = (g.gyro.x - gyroOffX) * RAD_TO_DEG;
    gyroY = (g.gyro.y - gyroOffY) * RAD_TO_DEG;
    gyroZ = (g.gyro.z - gyroOffZ) * RAD_TO_DEG;

    accX = a.acceleration.x;
    accY = a.acceleration.y;
    accZ = a.acceleration.z;
}

// ============================================================
//  Write same PWM to all motors
// ============================================================
void allMotors(int us) {
  m1.writeMicroseconds(us);
  m2.writeMicroseconds(us);
  m3.writeMicroseconds(us);
  m4.writeMicroseconds(us);
}
