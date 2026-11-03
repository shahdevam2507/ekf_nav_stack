#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <Arduino.h>

/* ================= GPIO DEFINITIONS ================= */

// LEFT MOTOR
const int pwml = 14;
const int dirl = 25;

// RIGHT MOTOR
const int pwmr = 17;
const int dirr = 21;

const int ledb = 2;

/* ================= micro-ROS OBJECTS ================= */

rcl_subscription_t subscriber;
rcl_publisher_t    pwm_publisher;
rclc_executor_t    executor;
rcl_allocator_t    allocator;
rclc_support_t     support;
rcl_node_t         node;

geometry_msgs__msg__Twist          cmd_msg;
std_msgs__msg__Int16MultiArray     pwm_msg;

/* ================= ERROR HANDLING ================= */

#define RCCHECK(fn)     { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) {} }

void error_loop()
{
  while (1) {
    digitalWrite(ledb, HIGH);
    delay(100);
    digitalWrite(ledb, LOW);
    delay(100);
  }
}

/* ================= MOTOR CONTROL ================= */

void controlmotor(float lin, float ang)
{
  int lspeed = (int)(255 * constrain(lin - ang, -1.0, 1.0));
  int rspeed = (int)(255 * constrain(lin + ang, -1.0, 1.0));

  // LEFT MOTOR
  digitalWrite(dirl, lspeed >= 0);
  ledcWrite(0, abs(lspeed));

  // RIGHT MOTOR
  digitalWrite(dirr, rspeed >= 0);
  ledcWrite(3, abs(rspeed));

  pwm_msg.data.data[0] = lspeed;
  pwm_msg.data.data[1] = rspeed;

  RCSOFTCHECK(rcl_publish(&pwm_publisher, &pwm_msg, NULL));
}

/* ================= ROS CALLBACK ================= */

void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg =
    (const geometry_msgs__msg__Twist *)msgin;

  controlmotor(msg->linear.x, msg->angular.z);
}

/* ================= SETUP ================= */

void setup()
{
  Serial.begin(115200);
  delay(2000);

  pinMode(ledb, OUTPUT);
  digitalWrite(ledb, HIGH);

  // Direction pins — one per motor
  pinMode(dirl, OUTPUT);
  pinMode(dirr, OUTPUT);

  /* ===== PWM SETUP (ONE CHANNEL PER PIN) ===== */
  ledcSetup(0, 5000, 8); ledcAttachPin(pwml, 0);   // Left motor  -> channel 0
  ledcSetup(3, 5000, 8); ledcAttachPin(pwmr, 3);   // Right motor -> channel 3

  /* ===== micro-ROS TRANSPORT ===== */
  set_microros_transports();

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "drive", "", &support));

  // Subscriber: /cmd_vel1 -> geometry_msgs/Twist
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel1"));

  // Publisher: /pwm_array -> std_msgs/Int16MultiArray
  RCCHECK(rclc_publisher_init_default(
    &pwm_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/pwm_array"));

  // Allocate storage for PWM feedback array [lspeed, rspeed]
  pwm_msg.data.data     = (int16_t *)malloc(2 * sizeof(int16_t));
  pwm_msg.data.capacity = 2;
  pwm_msg.data.size     = 2;

  // Executor: 1 handle (the subscription)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &cmd_msg,
    &subscription_callback, ON_NEW_DATA));

  digitalWrite(ledb, LOW);   // node started
}

/* ================= LOOP ================= */

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

