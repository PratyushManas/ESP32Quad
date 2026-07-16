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

float ReceiverValue[4];

// Desired angular rates from outer loop
float desiredRateRoll = 0;
float desiredRatePitch = 0;
float desiredRateYaw = 0;

// Actual PID outputs to motors
float inputRoll = 0;
float inputPitch = 0;
float inputYaw = 0;

uint32_t previousTime = 0;
float m1_pwm, m2_pwm, m3_pwm, m4_pwm;
float dt = 0.0;

const float alpha = 0.991f;

String input;

//variables for PID control
    float yaw_pid      = 0;
    float pitch_pid    = 0;
    float roll_pid     = 0;
float currentRoll, currentPitch, currentYaw;

float errRoll, errPitch;
float errSumRoll = 0, errSumPitch = 0;
float prevErrRoll = 0, prevErrPitch = 0;

// ---------- Angle PID ----------
float KpAngleR = 30.0, KiAngleR = 6.0, KdAngleR = 0.5;
float KpAngleP = 5.0, KiAngleP = 1.0, KdAngleP = 0.5;

// ---------- Rate PID ----------
float KpRateR = 0.0, KiRateR = 0.0, KdRateR = 0.0;
float KpRateP = 0.0, KiRateP = 0.0, KdRateP = 0.0;

// ---------- Yaw Rate PID ----------
float KpRateY = 0.0;
float KiRateY = 0.0;
float KdRateY = 0.0;

float rateErrSumYaw = 0.0;
float prevRateErrYaw = 0.0;

static float rateErrSumRoll = 0;
static float prevRateErrRoll = 0;

static float rateErrSumPitch = 0;
static float prevRateErrPitch = 0;

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
const uint32_t TELEMETRY_INTERVAL_MS    = 50;   // 5 Hz telemetry rate
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

        static uint32_t loopStart = micros();

        while ((micros() - loopStart) < 4000)
        {
        }

        loopStart += 4000;

  // 1. Read UDP (also detects new connection + remembers phone IP)
       readUDP();

        ReceiverValue[0] = 1500 + cmdRoll * 10.0f;     // Roll
        ReceiverValue[1] = 1500 + cmdPitch * 10.0f;    // Pitch
        ReceiverValue[2] = 1000 + cmdThrust * 1000.0f; // Throttle
        ReceiverValue[3] = 1500 + cmdYaw / 0.15f;      // Yaw

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
      if (!connected || cmdThrust < 0.05f)
      {
          allMotors(ESC_MIN);

          // ---------- Reset OUTER (angle) PID ----------
          errSumRoll  = 0.0f;
          errSumPitch = 0.0f;

          prevErrRoll  = 0.0f;
          prevErrPitch = 0.0f;

          // ---------- Reset INNER (rate) PID ----------
          rateErrSumRoll  = 0.0f;
          rateErrSumPitch = 0.0f;
          rateErrSumYaw   = 0.0f;

          prevRateErrRoll  = 0.0f;
          prevRateErrPitch = 0.0f;
          prevRateErrYaw   = 0.0f;

          // Reset yaw integration
          currentYaw = 0.0f;

          return;
      }
          
        updateAttitude();

  //calculating error for PID computation - IMU values are already read in this loop so no need to call it again
        calculateError();

  //calling the PID computation and writing the motor values int the function itself, only if the above if() clause is false.
        computePID();


    // Write motors here
    m1.writeMicroseconds((int)m1_pwm);
    m2.writeMicroseconds((int)m2_pwm);
    m3.writeMicroseconds((int)m3_pwm);
    m4.writeMicroseconds((int)m4_pwm);

  //Updating PID coefficients
if (Serial.available())
{
    input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "PID")
        printPIDValues();
    else
        processCommand(input);
}


  // // 9. Debug every 500ms (Serial only)
  //       static uint32_t lastDbg = 0;
  //       if (millis() - lastDbg > 500) {
  //         lastDbg = millis();
  //         Serial.printf("[FC] T:%.2f R:%.1f P:%.1f Y:%.1f",
  //                       cmdThrust, cmdRoll, cmdPitch, cmdYaw);
  //       }
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

  cmdThrust = constrain(cmdThrust, 0.0f, 0.8f);

  // Clamp for safety
  cmdRoll   = constrain(cmdRoll,   -30.0f,  30.0f);
  cmdPitch  = constrain(cmdPitch,  -30.0f,  30.0f);
  cmdYaw    = constrain(cmdYaw,   -100.0f, 100.0f);
  cmdThrust = constrain(cmdThrust,  0.0f,   0.80f);

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

    // char msg[160];
    // snprintf(msg, sizeof(msg),
    //          "THR:%d%% R:%.1f P:%.1f Y:%.1f GX:%.2f GY:%.2f GZ:%.2f AX:%.2f AY:%.2f AZ:%.2f",
    //          throttlePct, cmdRoll, cmdPitch, cmdYaw,
    //          gyroX, gyroY, gyroZ, accX, accY, accZ);


    //Modified Telemetry

    char msg[256];

    snprintf(msg, sizeof(msg),
        "T:%lu "
        "THR:%d "
        "CMD[R:%.1f P:%.1f Y:%.1f] "
        "ATT[R:%.2f P:%.2f Y:%.2f] "
        "GYR[X:%.2f Y:%.2f Z:%.2f] "
        "PID[R:%.1f P:%.1f Y:%.1f] "
        "M[%d %d %d %d]",

        millis(),
        throttlePct,
        cmdRoll,
        cmdPitch,
        cmdYaw,
        currentRoll,
        currentPitch,
        currentYaw,
        gyroX,
        gyroY,
        gyroZ,
        roll_pid,
        pitch_pid,
        yaw_pid,
        m1_pwm,
        m2_pwm,
        m3_pwm,
        m4_pwm
    );
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

    dt = 0.004f;

    //-----------------------------------
    // Accelerometer angles
    //-----------------------------------

    rollAcc = atan2(accY, sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;

    pitchAcc = -atan2(accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

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
void calculateError()
{
    errRoll  = cmdRoll  - currentRoll;
    errPitch = cmdPitch - currentPitch;
}


//Actual PID computation
void computePID() {
    float deltaErrRoll = 0, deltaErrPitch = 0, deltaErrYaw = 0;          // Error deltas in that order   : Yaw, Pitch, Roll

      // ---------- OUTER ANGLE PID ----------

      // Roll
      errSumRoll += errRoll * dt;
      errSumRoll = constrain(errSumRoll, -100, 100);

      deltaErrRoll = (errRoll - prevErrRoll) / dt;
      prevErrRoll = errRoll;

      //not using the cascaded PID for now
      // desiredRateRoll = KpAngleR * errRoll + KiAngleR * errSumRoll + KdAngleR * deltaErrRoll;

      roll_pid = (KpAngleR * errRoll) + (KiAngleR * errSumRoll) + (KdAngleR * deltaErrRoll);

      // Pitch
      errSumPitch += errPitch * dt;
      errSumPitch = constrain(errSumPitch, -100, 100);

      deltaErrPitch = (errPitch - prevErrPitch) / dt;
      prevErrPitch = errPitch;

      // not using the cascaded PID for now
      // desiredRatePitch = KpAngleP * errPitch + KiAngleP * errSumPitch + KdAngleP * deltaErrPitch;

      pitch_pid = (KpAngleP * errPitch) + (KiAngleP * errSumPitch) + (KdAngleP * deltaErrPitch);
      
      // desiredRateRoll  = constrain(desiredRateRoll,  -400.0f, 400.0f);
      // desiredRatePitch = constrain(desiredRatePitch, -400.0f, 400.0f);

      roll_pid  = constrain(roll_pid,  -400.0f, 400.0f);
      pitch_pid = constrain(pitch_pid, -400.0f, 400.0f);


      // // ---------- INNER RATE PID ----------

      // // Roll
      // float rateErrorRoll = desiredRateRoll - gyroX;

      // rateErrSumRoll += rateErrorRoll * dt;
      // rateErrSumRoll = constrain(rateErrSumRoll, -100, 100);

      // float dRateRoll = (rateErrorRoll - prevRateErrRoll) / dt;
      // prevRateErrRoll = rateErrorRoll;

      // roll_pid = KpRateR * rateErrorRoll + KiRateR * rateErrSumRoll + KdRateR * dRateRoll;


      // // Pitch
      // float rateErrorPitch = desiredRatePitch - gyroY;

      // rateErrSumPitch += rateErrorPitch * dt;
      // rateErrSumPitch = constrain(rateErrSumPitch, -100, 100);

      // float dRatePitch = (rateErrorPitch - prevRateErrPitch) / dt;
      // prevRateErrPitch = rateErrorPitch;

      // pitch_pid = KpRateP * rateErrorPitch + KiRateP * rateErrSumPitch + KdRateP * dRatePitch;

      // ---------- YAW RATE PID ----------

      float rateErrorYaw = cmdYaw - gyroZ;

      rateErrSumYaw += rateErrorYaw * dt;
      rateErrSumYaw = constrain(rateErrSumYaw, -100.0f, 100.0f);

      float dRateYaw = (rateErrorYaw - prevRateErrYaw) / dt;
      prevRateErrYaw = rateErrorYaw;

      yaw_pid = KpRateY * rateErrorYaw + KiRateY * rateErrSumYaw + KdRateY * dRateYaw;

      //  Motor Mixing
      float throttle = 1000.0f + cmdThrust * 1000.0f;

      m1_pwm = throttle + roll_pid - pitch_pid + yaw_pid;
      m2_pwm = throttle - roll_pid - pitch_pid - yaw_pid;
      m3_pwm = throttle - roll_pid + pitch_pid + yaw_pid;
      m4_pwm = throttle + roll_pid + pitch_pid - yaw_pid;

      m1_pwm = constrain(m1_pwm, ESC_MIN, ESC_MAX);
      m2_pwm = constrain(m2_pwm, ESC_MIN, ESC_MAX);
      m3_pwm = constrain(m3_pwm, ESC_MIN, ESC_MAX);
      m4_pwm = constrain(m4_pwm, ESC_MIN, ESC_MAX);

    // ---------- Serial Plotter ----------
    Serial.print(cmdRoll);
    Serial.print('\t');
    Serial.print(currentRoll);
    Serial.print('\t');
    Serial.println(gyroX);
}


void processCommand(String cmd)
{
    int eq = cmd.indexOf('=');

    if (eq == -1)
        return;

    String name = cmd.substring(0, eq);
    float value = cmd.substring(eq + 1).toFloat();

    // ---------- Roll Angle ----------
    if (name == "KpAR") KpAngleR = value;
    else if (name == "KiAR") KiAngleR = value;
    else if (name == "KdAR") KdAngleR = value;

    // ---------- Pitch Angle ----------
    else if (name == "KpAP") KpAngleP = value;
    else if (name == "KiAP") KiAngleP = value;
    else if (name == "KdAP") KdAngleP = value;

    // ---------- Roll Rate ----------
    else if (name == "KpRR") KpRateR = value;
    else if (name == "KiRR") KiRateR = value;
    else if (name == "KdRR") KdRateR = value;

    // ---------- Pitch Rate ----------
    else if (name == "KpRP") KpRateP = value;
    else if (name == "KiRP") KiRateP = value;
    else if (name == "KdRP") KdRateP = value;

    // ---------- Yaw Rate ----------
    else if (name == "KpY") KpRateY = value;
    else if (name == "KiY") KiRateY = value;
    else if (name == "KdY") KdRateY = value;

    else
    {
        Serial.println("Unknown parameter");
        return;
    }

    Serial.print(name);
    Serial.print(" = ");
    Serial.println(value, 6);
}

//Printing PID values on demand
void printPIDValues()
{
    Serial.println();
    Serial.println("========== CURRENT PID VALUES ==========");

    Serial.println();
    Serial.println("----- ANGLE PID -----");

    Serial.print("Roll : ");
    Serial.print(KpAngleR,4);
    Serial.print("  ");
    Serial.print(KiAngleR,4);
    Serial.print("  ");
    Serial.println(KdAngleR,4);

    Serial.print("Pitch: ");
    Serial.print(KpAngleP,4);
    Serial.print("  ");
    Serial.print(KiAngleP,4);
    Serial.print("  ");
    Serial.println(KdAngleP,4);

    Serial.println();
    Serial.println("----- RATE PID -----");

    Serial.print("Roll : ");
    Serial.print(KpRateR,4);
    Serial.print("  ");
    Serial.print(KiRateR,4);
    Serial.print("  ");
    Serial.println(KdRateR,4);

    Serial.print("Pitch: ");
    Serial.print(KpRateP,4);
    Serial.print("  ");
    Serial.print(KiRateP,4);
    Serial.print("  ");
    Serial.println(KdRateP,4);

    Serial.print("Yaw  : ");
    Serial.print(KpRateY,4);
    Serial.print("  ");
    Serial.print(KiRateY,4);
    Serial.print("  ");
    Serial.println(KdRateY,4);

    Serial.println("========================================");
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
