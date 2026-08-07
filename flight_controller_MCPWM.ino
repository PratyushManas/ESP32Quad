/*
 * ============================================================
 *  ESP32 Flight Controller v5  (+ telemetry, + gated attitude filter)
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
 *    - Adafruit MPU6050
 *    - Adafruit Unified Sensor
 *    - Adafruit BusIO
 *  (ESC PWM is generated with the native ESP32 MCPWM peripheral —
 *   no ESP32Servo dependency)
 * ============================================================
 */

#include <Wire.h>
#include "driver/mcpwm.h"
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

// ============================================================
//  MCPWM config for ESC PWM (replaces LEDC)
// ============================================================
// Standard ESC control signal is 50Hz with a 1000-2000us pulse.
// mcpwm_set_duty_in_us() takes the pulse width directly in
// microseconds and applies it immediately on the next PWM period
// — no separate "update" call needed (unlike LEDC's
// ledc_set_duty() + ledc_update_duty() two-step).
#define MCPWM_UNIT    MCPWM_UNIT_0
#define MCPWM_FREQ_HZ 50

const int motorPins[4] = { M1_PIN, M2_PIN, M3_PIN, M4_PIN };

// Unit 0 has 2 timers (0,1) x 2 generators (A,B) = 4 independent
// outputs, exactly enough for 4 motors.
const mcpwm_io_signals_t motorSignals[4] = { MCPWM0A, MCPWM0B, MCPWM1A, MCPWM1B };
const mcpwm_timer_t      motorTimers[4]  = { MCPWM_TIMER_0, MCPWM_TIMER_0, MCPWM_TIMER_1, MCPWM_TIMER_1 };
const mcpwm_generator_t  motorGens[4]    = { MCPWM_GEN_A, MCPWM_GEN_B, MCPWM_GEN_A, MCPWM_GEN_B };

// Write a microsecond pulse width to the given motor's generator
// (0-3, indexes motorTimers/motorGens). Timer/generator must
// already be configured via mcpwm_init()/mcpwm_gpio_init() in setup().
void writeMotorUs(int motorIndex, int us) {
  us = constrain(us, ESC_MIN, ESC_MAX);
  mcpwm_set_duty_in_us(MCPWM_UNIT, motorTimers[motorIndex], motorGens[motorIndex], us);
}


#define A_LPF_CUTOFF 1.0f    // Hz

static constexpr float beta_a_lpf =
    (2.0f * 3.14f * A_LPF_CUTOFF) /
    ((2.0f * 3.14f * A_LPF_CUTOFF) + 250.0f);

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

// Real elapsed time between attitude updates, measured with micros()
// instead of assuming the loop always takes exactly 4ms. If a loop
// iteration ever overruns (e.g. a blocking Serial.print), dt reflects
// what actually happened instead of silently under-integrating the gyro.
uint32_t lastAttitudeUpdateUs = 0;
const float DT_NOMINAL   = 0.004f;  // 4ms target loop period
const float DT_MIN       = 0.001f;  // clamp bounds so a hiccup / first
const float DT_MAX       = 0.020f;  // call can't produce a garbage dt

// ---- Attitude filter tuning ----
// Weight given to the gyro-integrated angle vs. the accelerometer angle
// in the complementary filter. Closer to 1.0 = trust gyro more (smoother,
// slightly more drift). Closer to 0.0 = trust accel more (noisier, no drift).
// NOTE: this replaces the old `alpha`/`beta` pair, which effectively made
// the filter pure-accelerometer (alpha was 0, so the gyro integration was
// being thrown away every loop).
const float compFilterAlpha = 0.98f;

// Continuously-updated gyro bias estimate, refined on top of the static
// calibration captured at boot (gyroOffX/Y/Z). Corrects slow bias drift
// that the one-shot startup calibration can't track.
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;

// Only trust the accelerometer's angle estimate when the measured
// acceleration magnitude is close to gravity (i.e. the vehicle isn't
// under significant linear acceleration/vibration). Same idea as the
// low-pass-filter test code's `filter_update()` gating.
const float ACC_MAG_TRUST_BAND = 1.0f; // m/s^2 tolerance around 9.81

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
float KpAngleR = 3.0, KiAngleR = 0.0, KdAngleR = 0.3;
float KpAngleP = 0.0, KiAngleP = 0.0, KdAngleP = 0.0;

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
float wrap180(float angle);

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Flight Controller v5 (+telemetry, +gated filter) ===");

  // ── 1. Arm ESCs ────────────────────────────────────────
            for (int i = 0; i < 4; i++) {
              mcpwm_gpio_init(MCPWM_UNIT, motorSignals[i], motorPins[i]);
            }

            mcpwm_config_t pwmConfig;
            pwmConfig.frequency    = MCPWM_FREQ_HZ;
            pwmConfig.cmpr_a       = 0;
            pwmConfig.cmpr_b       = 0;
            pwmConfig.counter_mode = MCPWM_UP_COUNTER;
            pwmConfig.duty_mode    = MCPWM_DUTY_MODE_0;

            mcpwm_init(MCPWM_UNIT, MCPWM_TIMER_0, &pwmConfig);
            mcpwm_init(MCPWM_UNIT, MCPWM_TIMER_1, &pwmConfig);

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
            mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
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


//     // Write motors here
// static uint32_t maxM1=0, maxM2=0, maxM3=0, maxM4=0;
// uint32_t mt0=micros();

//     writeMotorUs(0, (int)m1_pwm); uint32_t mt1=micros(); 
//     writeMotorUs(1, (int)m2_pwm); uint32_t mt2=micros(); 
//     writeMotorUs(2, (int)m3_pwm); uint32_t mt3=micros();
//     writeMotorUs(3, (int)m4_pwm); uint32_t mt4=micros();

// if (mt1-mt0 > maxM2) maxM2 = mt1-mt0;
// if (mt2-mt1 > maxM1) maxM1 = mt2-mt1;
// if (mt3-mt2 > maxM3) maxM3 = mt3-mt2;
// if (mt4-mt3 > maxM4) maxM4 = mt4-mt3;

//         //motor debug
// static uint32_t maxM1=0, maxM2=0, maxM3=0, maxM4=0;
// uint32_t mt0=micros();

//     writeMotorUs(M1_PIN, (int)m1_pwm); uint32_t mt1=micros();
//     writeMotorUs(M2_PIN, (int)m2_pwm); uint32_t mt2=micros();
//     writeMotorUs(M3_PIN, (int)m3_pwm); uint32_t mt3=micros();
//     writeMotorUs(M4_PIN, (int)m4_pwm); uint32_t mt4=micros();

// if (mt1-mt0 > maxM2) maxM1 = mt1-mt0;
// if (mt2-mt1 > maxM1) maxM2 = mt2-mt1;
// if (mt3-mt2 > maxM3) maxM3 = mt3-mt2;
// if (mt4-mt3 > maxM4) maxM4 = mt4-mt3;


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


    //debug
    // static uint32_t dbgLastReportMs = 0;
    // if (millis() - dbgLastReportMs > 1000) {
    //   dbgLastReportMs = millis();
    //   Serial.printf("maxM1:%lu maxM2:%lu maxM3:%lu maxM4:%lu\n", maxM1, maxM2, maxM3, maxM4);
    //   maxM1=maxM2=maxM3=maxM4=0;
    // }

    // // Write motors here
    // static uint32_t maxM1=0, maxM2=0, maxM3=0, maxM4=0;
    // uint32_t mt0=micros();

    writeMotorUs(0, (int)m1_pwm);
    writeMotorUs(1, (int)m2_pwm);
    writeMotorUs(2, (int)m3_pwm);
    writeMotorUs(3, (int)m4_pwm);
    // uint32_t t_mt1 = mt1 - mt0, t_mt2 = mt2 - mt1, t_mt3 = mt3 - mt2, t_mt4 = mt4 - mt3;
    // uint32_t total = t_mt1 + t_mt2 + t_mt3 + t_mt4;

    // Serial.printf("Time taken for writing motors: {%lu, %lu, %lu, %lu}, total = %lu\n", t_mt1, t_mt2, t_mt3, t_mt4, total);
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

    char msg[160];
    snprintf(msg, sizeof(msg),
             "THR:%d%% R:%.1f P:%.1f Y:%.1f GX:%.2f GY:%.2f GZ:%.2f AX:%.2f AY:%.2f AZ:%.2f",
             throttlePct, cmdRoll, currentRoll, cmdYaw,
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

// ============================================================
//  Wrap an angle in degrees to the range [-180, 180]
// ============================================================
float wrap180(float angle) {
  while (angle > 180.0f)  angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

// ============================================================
//  Attitude calculation — gated, bias-correcting complementary filter
//  (ported from the ImuFilter test code, adapted to degrees and to
//  this project's existing globals instead of a Vec3/struct API)
// ============================================================
void updateAttitude()
{
    // ---- Measure real elapsed time instead of assuming 4ms ----
    uint32_t nowUs = micros();
    if (lastAttitudeUpdateUs == 0) {
        dt = DT_NOMINAL;               // first call after arming: use nominal
    } else {
        dt = (nowUs - lastAttitudeUpdateUs) / 1000000.0f;
        dt = constrain(dt, DT_MIN, DT_MAX);  // guard against a stalled loop
    }
    lastAttitudeUpdateUs = nowUs;

    // Angles from the previous loop, before this update overwrites them —
    // needed for the gyro-bias refinement below.
    float prevRoll  = currentRoll;
    float prevPitch = currentPitch;

    //-----------------------------------
    // Predict: integrate bias-corrected gyro
    //-----------------------------------
    float g_r = wrap180(prevRoll  + (gyroX - gyroBiasX) * dt);
    float g_p = wrap180(prevPitch + (gyroY - gyroBiasY) * dt);
    float g_y = wrap180(currentYaw + gyroZ * dt);

    //-----------------------------------
    // Only correct with the accelerometer when the measured
    // acceleration is close to 1g, i.e. the vehicle isn't under
    // significant linear acceleration or vibration. Otherwise the
    // accelerometer reading is unreliable and we trust the gyro alone.
    //-----------------------------------
    float accMag = sqrt(accX * accX + accY * accY + accZ * accZ);

    if (fabs(accMag - 9.81f) < ACC_MAG_TRUST_BAND) {
        rollAcc  = atan2(accY, accZ) * RAD_TO_DEG;
        pitchAcc = -atan2(accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

        // Complementary filter fusion
        currentRoll  = (compFilterAlpha * g_r) + ((1.0f - compFilterAlpha) * rollAcc);
        currentPitch = (compFilterAlpha * g_p) + ((1.0f - compFilterAlpha) * pitchAcc);
        currentYaw   = g_y;

        // Refine the continuous gyro bias estimate from how much the
        // accelerometer just corrected the gyro-only prediction.
        gyroBiasX = gyroX - ((currentRoll  - prevRoll)  / dt);
        gyroBiasY = gyroY - ((currentPitch - prevPitch) / dt);
    } else {
        // Under linear acceleration / vibration: gyro-only prediction.
        currentRoll  = g_r;
        currentPitch = g_p;
        currentYaw   = g_y;
    }
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

      roll_pid  = constrain(roll_pid,  -300.0f, 300.0f);
      pitch_pid = constrain(pitch_pid, -300.0f, 300.0f);


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

    // ---------- Serial Plotter (throttled) ----------
    // NOTE: this used to run every loop at 250Hz. At 115200 baud that's
    // enough traffic to fill the TX buffer and make Serial.print() block
    // for several ms whenever nothing is reading it fast enough — which
    // silently stretched the loop period while `dt` still assumed 4ms,
    // making the attitude estimate lag behind real motion. Throttled to
    // 20Hz here; raise/lower PLOTTER_INTERVAL_MS as needed, or comment
    // the block out entirely once you're done tuning.
    static uint32_t lastPlotMs = 0;
    const uint32_t PLOTTER_INTERVAL_MS = 50; // 20Hz
    if (millis() - lastPlotMs >= PLOTTER_INTERVAL_MS) {
        lastPlotMs = millis();
        Serial.print(cmdRoll);
        Serial.print('\t');
        Serial.print(currentRoll);
        Serial.print('\t');
        Serial.println(gyroX);
    }
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

    gyroX = (g.gyro.x - gyroOffX);
    gyroY = (g.gyro.y - gyroOffY);
    gyroZ = (g.gyro.z - gyroOffZ);

    accX += beta_a_lpf * (a.acceleration.x - accX);
    accY += beta_a_lpf * (a.acceleration.y - accY);
    accZ += beta_a_lpf * (a.acceleration.z - accZ);
}

// ============================================================
//  Write same PWM to all motors
// ============================================================
void allMotors(int us) {
  for (int i = 0; i < 4; i++) writeMotorUs(i, us);
}
