#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 2

// Motor control definitions
#define PIN_IN1 16
#define PIN_IN2 17
#define PIN_IN3 5
#define PIN_IN4 18
#define PIN_ENA_LEFT_F 19
#define PIN_ENA_RIGHT_F 4

#define PIN_IN5 25
#define PIN_IN6 26
#define PIN_IN7 27
#define PIN_IN8 14
#define PIN_ENA_LEFT_B 33
#define PIN_ENA_RIGHT_B 12

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void controlMotors(int command, int speed = 255) {
  if (command == 1) { // forward
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    analogWrite(PIN_ENA_LEFT_F, speed);
    analogWrite(PIN_ENA_RIGHT_F, speed);

    digitalWrite(PIN_IN5, HIGH);
    digitalWrite(PIN_IN6, LOW);
    digitalWrite(PIN_IN7, LOW);
    digitalWrite(PIN_IN8, HIGH);
    analogWrite(PIN_ENA_LEFT_B, speed);
    analogWrite(PIN_ENA_RIGHT_B, speed);
  } else if (command == 2) { // backward
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENA_LEFT_F, speed);
    analogWrite(PIN_ENA_RIGHT_F, speed);

    digitalWrite(PIN_IN5, LOW);
    digitalWrite(PIN_IN6, HIGH);
    digitalWrite(PIN_IN7, HIGH);
    digitalWrite(PIN_IN8, LOW);
    analogWrite(PIN_ENA_LEFT_B, speed);
    analogWrite(PIN_ENA_RIGHT_B, speed);
  } else if (command == 3) { // left
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    analogWrite(PIN_ENA_LEFT_F, 0);
    analogWrite(PIN_ENA_RIGHT_F, speed);

    digitalWrite(PIN_IN5, LOW);
    digitalWrite(PIN_IN6, HIGH);
    digitalWrite(PIN_IN7, LOW);
    digitalWrite(PIN_IN8, HIGH);
    analogWrite(PIN_ENA_LEFT_B, 0);
    analogWrite(PIN_ENA_RIGHT_B, speed);
  } else if (command == 4) { // right
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENA_LEFT_F, speed);
    analogWrite(PIN_ENA_RIGHT_F, 0);

    digitalWrite(PIN_IN5, HIGH);
    digitalWrite(PIN_IN6, LOW);
    digitalWrite(PIN_IN7, HIGH);
    digitalWrite(PIN_IN8, LOW);
    analogWrite(PIN_ENA_LEFT_B, speed);
    analogWrite(PIN_ENA_RIGHT_B, 0);
  } else { // "stop" or any other command
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENA_LEFT_F, 0);
    analogWrite(PIN_ENA_RIGHT_F, 0);

    digitalWrite(PIN_IN5, LOW);
    digitalWrite(PIN_IN6, LOW);
    digitalWrite(PIN_IN7, LOW);
    digitalWrite(PIN_IN8, LOW);
    analogWrite(PIN_ENA_LEFT_B, 0);
    analogWrite(PIN_ENA_RIGHT_B, 0);
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  // digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
  // 1 - Forward
  // 2 - Backward
  // 3 - Left
  // 4 - Right
  // 5 - Stop
  controlMotors(msg->data);
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_IN5, OUTPUT);
  pinMode(PIN_IN6, OUTPUT);
  pinMode(PIN_IN7, OUTPUT);
  pinMode(PIN_IN8, OUTPUT);
  controlMotors(0); // Stop motors initially

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_subscriber"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}