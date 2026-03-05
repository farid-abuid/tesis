#include <micro_ros_arduino.h>

#include <FlexCAN_T4.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float64_multi_array.h>

#define LED_PIN 13
#define NUM_MOTORS 1
#define CONTROL_DT_US 500  // 2 kHz

// ROS topics
#define CMD_TOPIC "/torque_commands"
#define STATUS_TOPIC "/joint_states"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
uint8_t motorIDs[NUM_MOTORS] = {3};

struct __attribute__((packed)) MotorStatus2 {
  int8_t motorTemp;
  int16_t iq;     // 0.01 A/LSB
  int16_t speed;  // dps
  int16_t angle;  // degree
};

static double motorCmd[NUM_MOTORS] = {0.0};
static MotorStatus2 motorData[NUM_MOTORS];

// Subscriber static storage
static double cmd_buffer[NUM_MOTORS];

// JointState static storage
static double joint_pos_buffer[NUM_MOTORS];
static double joint_vel_buffer[NUM_MOTORS];
static double joint_eff_buffer[NUM_MOTORS];

rcl_node_t node;
rcl_publisher_t status_pub;
rcl_subscription_t cmd_sub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_timer_t control_timer;

std_msgs__msg__Float64MultiArray cmd_msg;
sensor_msgs__msg__JointState status_msg;

uint32_t last_control_tick = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

static inline uint32_t motorCmdID(uint8_t motorID) { return 0x140u + motorID; }

int motorIndexFromID(uint8_t id) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorIDs[i] == id) return i;
  }
  return -1;
}

void canTorqueControl(uint8_t motorID, int16_t iqControl) {
  CAN_message_t msg{};
  msg.id = motorCmdID(motorID);
  msg.len = 8;

  // 0xA1 is the motor protocol command for torque/current control
  msg.buf[0] = 0xA1;
  msg.buf[1] = 0x00;
  msg.buf[2] = 0x00;
  msg.buf[3] = 0x00;
  msg.buf[4] = (uint8_t)(iqControl);
  msg.buf[5] = (uint8_t)(iqControl >> 8);
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;

  Can1.write(msg);
}

bool canParseTorqueControlReply(const CAN_message_t &msg, MotorStatus2 &out) {
  // Reply to torque/current control uses command byte 0xA1 in byte 0
  if (msg.buf[0] != 0xA1) return false;

  out.motorTemp = (int8_t)msg.buf[1];
  out.iq = (int16_t)(msg.buf[2] | (msg.buf[3] << 8));
  out.speed = (int16_t)(msg.buf[4] | (msg.buf[5] << 8));
  out.angle = (int16_t)(msg.buf[6] | (msg.buf[7] << 8));
  return true;
}

void processCANRx(bool receivedConfirm[NUM_MOTORS]) {
  CAN_message_t rx;
  MotorStatus2 status;

  while (Can1.read(rx)) {
    if (canParseTorqueControlReply(rx, status)) {
      int idx = motorIndexFromID((uint8_t)(rx.id - 0x240));
      if (idx >= 0) {
        motorData[idx] = status;
        receivedConfirm[idx] = true;
      }
    }
  }
}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void command_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *msg = (const std_msgs__msg__Float64MultiArray *)msgin;

  size_t n = msg->data.size;
  if (n > NUM_MOTORS) n = NUM_MOTORS;

  noInterrupts();
  for (size_t i = 0; i < n; i++) {
    motorCmd[i] = msg->data.data[i];
  }
  interrupts();
}

void control_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  uint32_t now = micros();
  if (now - last_control_tick < CONTROL_DT_US) return;
  last_control_tick += CONTROL_DT_US;

  for (int i = 0; i < NUM_MOTORS; i++) {
    canTorqueControl(motorIDs[i], (int16_t)(motorCmd[i] * 100.0));
  }

  bool receivedConfirm[NUM_MOTORS] = {};
  processCANRx(receivedConfirm);

  bool any_confirm = false;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (receivedConfirm[i]) {
      any_confirm = true;
      joint_pos_buffer[i] = (double)motorData[i].angle;
      joint_vel_buffer[i] = (double)motorData[i].speed;
      joint_eff_buffer[i] = ((double)motorData[i].iq) / 100.0;  // A
    }
  }

  if (!any_confirm) return;

  uint32_t ts_us = micros();
  status_msg.header.stamp.sec = (int32_t)(ts_us / 1000000u);
  status_msg.header.stamp.nanosec = (uint32_t)((ts_us % 1000000u) * 1000u);

  RCSOFTCHECK(rcl_publish(&status_pub, &status_msg, NULL));
}

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  Can1.begin();
  Can1.setBaudRate(1000000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_ros_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
      &status_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      STATUS_TOPIC));

  RCCHECK(rclc_subscription_init_default(
      &cmd_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      CMD_TOPIC));

  // Subscriber buffer
  cmd_msg.data.data = cmd_buffer;
  cmd_msg.data.size = 0;
  cmd_msg.data.capacity = NUM_MOTORS;
  cmd_msg.layout.dim.data = NULL;
  cmd_msg.layout.dim.size = 0;
  cmd_msg.layout.dim.capacity = 0;
  cmd_msg.layout.data_offset = 0;

  // JointState fixed arrays
  status_msg.name.data = NULL;
  status_msg.name.size = 0;
  status_msg.name.capacity = 0;

  status_msg.position.data = joint_pos_buffer;
  status_msg.position.size = NUM_MOTORS;
  status_msg.position.capacity = NUM_MOTORS;

  status_msg.velocity.data = joint_vel_buffer;
  status_msg.velocity.size = NUM_MOTORS;
  status_msg.velocity.capacity = NUM_MOTORS;

  status_msg.effort.data = joint_eff_buffer;
  status_msg.effort.size = NUM_MOTORS;
  status_msg.effort.capacity = NUM_MOTORS;

  RCCHECK(rclc_timer_init_default(
      &control_timer,
      &support,
      RCL_MS_TO_NS(1),
      control_timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &command_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  last_control_tick = micros();
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}
