#include <Arduino.h>
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"

// ─── micro-ROS ────────────────────────────────────────────────────────────────
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

rcl_subscription_t cmd_vel_sub;
rclc_executor_t    executor;
rcl_allocator_t    allocator;
rclc_support_t     support;
rcl_node_t         node;
geometry_msgs__msg__Twist cmd_msg;

#define RCCHECK(fn)     { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) {} }

// ─── ROBOT PHYSICAL PARAMETERS ───────────────────────────────────────────────
#define WHEEL_RADIUS_M  0.0367f   // r  [m]
#define TRACK_WIDTH_M   0.239f     // L  [m]  centre-to-centre axle width
#define ENCODER_PPR     3510.0f    // pulses / output-shaft revolution (1x)
#define MAX_MOTOR_RPM   60.0f

// ─── MOTOR 1 (LEFT) ──────────────────────────────────────────────────────────
#define M1_PWM_PIN  18
#define M1_DIR_PIN  13
#define M1_ENC_A    32
#define M1_ENC_B    33

// ─── MOTOR 2 (RIGHT) ─────────────────────────────────────────────────────────
#define M2_PWM_PIN  4
#define M2_DIR_PIN  15
#define M2_ENC_A    27
#define M2_ENC_B    26

// ─── ERROR LED ────────────────────────────────────────────────────────────────
#define LED_PIN     2

// ─── CONTROL LAW GAINS ───────────────────────────────────────────────────────
#define DEADZONE_L   8.0f
#define DEADZONE_R   11.0f

const float K_FF_L = (255.0f - DEADZONE_L) / MAX_MOTOR_RPM;
const float K_FF_R = (255.0f - DEADZONE_R) / MAX_MOTOR_RPM;

float Kp     = 2.0f;
float Ki     = 10.0f;
float K_sync = 0.25f;
#define INTEGRAL_MAX  60.0f
#define SYNC_MAX_PWM  30.0f

// ─── TIMING & PWM ────────────────────────────────────────────────────────────
#define PWM_FREQ        20000
#define PWM_RES         8
#define SAMPLE_TIME_MS  20
#define ZERO_TIMEOUT_US 150000UL

// ─── SAFETY WATCHDOG ─────────────────────────────────────────────────────────
#define CMD_VEL_TIMEOUT_US  500000UL   // stop if no cmd_vel for 500 ms

volatile uint32_t lastCmdVelUs = 0;
bool rosCmdActive = false;

// ─────────────────────────────────────────────────────────────────────────────

enum ControlMode { MODE_STOP, MODE_VELOCITY_PI, MODE_CALIB_REVS };
ControlMode currentMode = MODE_STOP;

struct WheelState {
  volatile long     pos         = 0;
  volatile uint32_t lastPulseUs = 0;
  volatile int      dir         = 1;

  long     prevPos    = 0;
  uint32_t prevSyncUs = 0;

  float rpm_MT       = 0.0f;
  float omega_rad_s  = 0.0f;

  float targetRpm    = 0.0f;
  float integralTerm = 0.0f;
  int   appliedPwm   = 0;

  long syncStartPos     = 0;
  long calibTargetTicks = 0;
  bool calibDone        = false;
};

WheelState encL, encR;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

float x_pos     = 0.0f;
float y_pos     = 0.0f;
float theta_rad = 0.0f;
float V_body    = 0.0f;
float W_body    = 0.0f;

uint32_t prevSampleTimeUs = 0;

// ─── ERROR LOOP ───────────────────────────────────────────────────────────────
void error_loop()
{
  while (1) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }
}

// ─── ISRs ─────────────────────────────────────────────────────────────────────
void IRAM_ATTR isrMotor1() {
  uint32_t nowUs = micros();
  int bState = (REG_READ(GPIO_IN1_REG) >> (M1_ENC_B - 32)) & 0x1;
  int d = (bState == 0) ? 1 : -1;
  portENTER_CRITICAL_ISR(&mux);
  encL.lastPulseUs = nowUs; encL.pos += d; encL.dir = d;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR isrMotor2() {
  uint32_t nowUs = micros();
  int bState = (REG_READ(GPIO_IN_REG) >> M2_ENC_B) & 0x1;
  int d = (bState == 0) ? 1 : -1;
  portENTER_CRITICAL_ISR(&mux);
  encR.lastPulseUs = nowUs; encR.pos += d; encR.dir = d;
  portEXIT_CRITICAL_ISR(&mux);
}

// ─── MOTOR OUTPUT ─────────────────────────────────────────────────────────────
void setMotors(int pwmL, int pwmR) {
  pwmL = constrain(pwmL, -255, 255);
  pwmR = constrain(pwmR, -255, 255);
  encL.appliedPwm = pwmL;
  encR.appliedPwm = pwmR;

  digitalWrite(M1_DIR_PIN, (pwmL >= 0) ? LOW : HIGH);
  digitalWrite(M2_DIR_PIN, (pwmR >= 0) ? LOW : HIGH);

  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(M1_PWM_PIN, abs(pwmL));
    ledcWrite(M2_PWM_PIN, abs(pwmR));
  #else
    ledcWrite(0, abs(pwmL));
    ledcWrite(1, abs(pwmR));
  #endif
}

void stopAll() {
  currentMode       = MODE_STOP;
  encL.targetRpm    = 0.0f;
  encR.targetRpm    = 0.0f;
  encL.integralTerm = 0.0f;
  encR.integralTerm = 0.0f;
  rosCmdActive      = false;
  setMotors(0, 0);
}

// ─── micro-ROS CALLBACK ───────────────────────────────────────────────────────
// Receives geometry_msgs/Twist from /cmd_vel1 (published by teleop.cpp).
//   linear.x  = vb  [m/s]
//   angular.z = w   [rad/s]
void cmd_vel_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;

  float vb = (float)msg->linear.x;
  float wb = (float)msg->angular.z;

  lastCmdVelUs = micros();
  rosCmdActive = true;

  long curL, curR;
  portENTER_CRITICAL(&mux);
  curL = encL.pos; curR = encR.pos;
  portEXIT_CRITICAL(&mux);

  // Differential drive: vb, w → wheel linear velocities
  //   v_L = vb + (L/2) * w
  //   v_R = vb - (L/2) * w
  float vL = vb - (0.5f * TRACK_WIDTH_M * wb); // make changes on this line if needed 
  float vR = vb + (0.5f * TRACK_WIDTH_M * wb);

  // Wheel linear velocity → RPM = (v / 2πr) * 60
  float newRpmL = (vL / (2.0f * PI * WHEEL_RADIUS_M)) * 60.0f;
  float newRpmR = (vR / (2.0f * PI * WHEEL_RADIUS_M)) * 60.0f;

  // Reset integral only on direction reversal
  if ((newRpmL * encL.targetRpm) < 0.0f) encL.integralTerm = 0.0f;
  if ((newRpmR * encR.targetRpm) < 0.0f) encR.integralTerm = 0.0f;

  encL.targetRpm    = newRpmL;
  encR.targetRpm    = newRpmR;
  encL.syncStartPos = curL;
  encR.syncStartPos = curR;

  currentMode = MODE_VELOCITY_PI;
}

// ─── PI + FEEDFORWARD ─────────────────────────────────────────────────────────
float computeWheelPI(WheelState &w, float deadzone, float k_ff, float dtSec)
{
  if (abs(w.targetRpm) < 0.05f) {
    w.integralTerm = 0.0f;
    return 0.0f;
  }
  float sign = (w.targetRpm > 0.0f) ? 1.0f : -1.0f;
  float u_ff = (deadzone * sign) + (k_ff * w.targetRpm);
  float err  = w.targetRpm - w.rpm_MT;
  w.integralTerm += Ki * err * dtSec;
  w.integralTerm  = constrain(w.integralTerm, -INTEGRAL_MAX, INTEGRAL_MAX);
  return u_ff + (Kp * err) + w.integralTerm;
}

// ─── SETUP ────────────────────────────────────────────────────────────────────
void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Encoder inputs
  pinMode(M1_ENC_A, INPUT_PULLUP); pinMode(M1_ENC_B, INPUT_PULLUP);
  pinMode(M2_ENC_A, INPUT_PULLUP); pinMode(M2_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isrMotor1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), isrMotor2, RISING);

  // Motor direction pins
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);

  // PWM channels
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(M1_PWM_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(M2_PWM_PIN, PWM_FREQ, PWM_RES);
  #else
    ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(M1_PWM_PIN, 0);
    ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(M2_PWM_PIN, 1);
  #endif

  // ── micro-ROS over USB (data cable) ──────────────────────────────────────
  set_microros_transports();   // uses default Serial over USB
  delay(2000);                 // wait for agent

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "drive_ctrl", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel1"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &cmd_vel_sub, &cmd_msg,
      &cmd_vel_callback, ON_NEW_DATA));

  stopAll();
  uint32_t now  = micros();
  prevSampleTimeUs = now;
  encL.prevSyncUs  = now;
  encR.prevSyncUs  = now;
  lastCmdVelUs     = now;

  digitalWrite(LED_PIN, LOW);    // ✅ node ready
}

// ─── LOOP ─────────────────────────────────────────────────────────────────────
void loop()
{
  // 1. Pump micro-ROS (fires cmd_vel_callback on new message)
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));

  // 2. Safety watchdog — stop if /cmd_vel1 goes silent
  if (rosCmdActive && (micros() - lastCmdVelUs) > CMD_VEL_TIMEOUT_US) {
    stopAll();
  }

  // 3. 50 Hz control + odometry loop
  uint32_t currentTimeUs = micros();
  if ((currentTimeUs - prevSampleTimeUs) < (SAMPLE_TIME_MS * 1000UL)) return;

  float dtSec = (currentTimeUs - prevSampleTimeUs) / 1000000.0f;

  long     posL, posR;
  uint32_t pulseUsL, pulseUsR;

  portENTER_CRITICAL(&mux);
  posL = encL.pos; pulseUsL = encL.lastPulseUs;
  posR = encR.pos; pulseUsR = encR.lastPulseUs;
  portEXIT_CRITICAL(&mux);

  long dTicksL = posL - encL.prevPos;
  long dTicksR = posR - encR.prevPos;

  // ── STEP A: M/T velocity estimation ──────────────────────────────────────
  bool     stoppedL = (currentTimeUs - pulseUsL) > ZERO_TIMEOUT_US;
  uint32_t syncUsL  = pulseUsL - encL.prevSyncUs;
  encL.rpm_MT = (!stoppedL && dTicksL != 0 && syncUsL > 0)
                ? (dTicksL * 60.0f * 1000000.0f) / (ENCODER_PPR * syncUsL) : 0.0f;
  encL.omega_rad_s = encL.rpm_MT * (2.0f * PI / 60.0f);

  bool     stoppedR = (currentTimeUs - pulseUsR) > ZERO_TIMEOUT_US;
  uint32_t syncUsR  = pulseUsR - encR.prevSyncUs;
  encR.rpm_MT = (!stoppedR && dTicksR != 0 && syncUsR > 0)
                ? (dTicksR * 60.0f * 1000000.0f) / (ENCODER_PPR * syncUsR) : 0.0f;
  encR.omega_rad_s = encR.rpm_MT * (2.0f * PI / 60.0f);

  V_body = (WHEEL_RADIUS_M * 0.5f) * (encL.omega_rad_s + encR.omega_rad_s);
  W_body = (WHEEL_RADIUS_M / TRACK_WIDTH_M) * (encR.omega_rad_s - encL.omega_rad_s);// make changes if the turn is inverted

  // ── STEP B: RK2 midpoint odometry ────────────────────────────────────────
  float distPerTick = (2.0f * PI * WHEEL_RADIUS_M) / ENCODER_PPR;
  float dL      = -dTicksL * distPerTick;
  float dR      = -dTicksR * distPerTick;
  float dCenter = 0.5f * (dR + dL);
  float dTheta  = (dR - dL) / TRACK_WIDTH_M; // make changes if the turn is inverted 
  float thetaMid = theta_rad + 0.5f * dTheta;

  x_pos     += dCenter * cosf(thetaMid);
  y_pos     += dCenter * sinf(thetaMid);
  theta_rad += dTheta;
  if (theta_rad >  PI) theta_rad -= 2.0f * PI;
  if (theta_rad < -PI) theta_rad += 2.0f * PI;

  // ── STEP C: PI + FF + tick-sync ──────────────────────────────────────────
  if (currentMode == MODE_VELOCITY_PI || currentMode == MODE_CALIB_REVS) {

    if (currentMode == MODE_CALIB_REVS) {
      long travL = abs(posL - encL.syncStartPos);
      long travR = abs(posR - encR.syncStartPos);
      if (travL >= encL.calibTargetTicks) { encL.targetRpm = 0.0f; encL.calibDone = true; }
      if (travR >= encR.calibTargetTicks) { encR.targetRpm = 0.0f; encR.calibDone = true; }
      if (encL.calibDone && encR.calibDone) stopAll();
    }

    float uL = computeWheelPI(encL, DEADZONE_L, K_FF_L, dtSec);
    float uR = computeWheelPI(encR, DEADZONE_R, K_FF_R, dtSec);

    // Straight-line tick sync
    if (abs(encL.targetRpm - encR.targetRpm) < 0.01f && abs(encL.targetRpm) > 0.05f) {
      long  deltaL   = posL - encL.syncStartPos;
      long  deltaR   = posR - encR.syncStartPos;
      float magDiff  = (float)(abs(deltaL) - abs(deltaR));
      float signL    = (encL.targetRpm > 0) ? 1.0f : -1.0f;
      float syncTrim = constrain(K_sync * magDiff, -SYNC_MAX_PWM, SYNC_MAX_PWM);
      uL -= syncTrim * signL;
      uR += syncTrim * signL;
    }
    // Spin-in-place tick sync
    else if (abs(encL.targetRpm + encR.targetRpm) < 0.01f && abs(encL.targetRpm) > 0.05f) {
      long  magL     = abs(posL - encL.syncStartPos);
      long  magR     = abs(posR - encR.syncStartPos);
      float magDiff  = (float)(magL - magR);
      float syncTrim = constrain(K_sync * magDiff, -SYNC_MAX_PWM, SYNC_MAX_PWM);
      float signL    = (encL.targetRpm > 0) ? 1.0f : -1.0f;
      float signR    = (encR.targetRpm > 0) ? 1.0f : -1.0f;
      uL -= syncTrim * signL;
      uR += syncTrim * signR;
    }

    setMotors((int)roundf(uL), (int)roundf(uR));
  }

  encL.prevPos = posL; encR.prevPos = posR;
  if (dTicksL != 0) encL.prevSyncUs = pulseUsL;
  if (dTicksR != 0) encR.prevSyncUs = pulseUsR;
  prevSampleTimeUs = currentTimeUs;
}

