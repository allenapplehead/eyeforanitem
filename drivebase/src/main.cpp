#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#include <micro_ros_arduino.h>
#include <basicMPU6050.h>
#include <sensor_msgs/msg/imu.h>

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

// Define Motor Directions
#define FWD 1
#define BCK 2
#define LEFT 3
#define RIGHT 4
#define STOP 5

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

// Create instance of MPU6050
basicMPU6050<> imu;

// ROS publisher and message
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;


void controlMotors(int command, int speed = 255) {
  if (command == FWD) { // forward
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
  } else if (command == BCK) { // backward
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
  } else if (command == LEFT) { // left
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
  } else if (command == RIGHT) { // right
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


// Setup function for micro-ROS
void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // Port definitions for motors (and LED for debugging)
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

  // Initialize MPU6050
  imu.setup();
  imu.setBias();

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_drivebase", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "drivebase_subscriber"));

  // Create IMU publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu"));

  // Executor initialization
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  // Update gyro calibration
  imu.updateBias();
 
  // Fill the IMU message
  imu_msg.angular_velocity.x = imu.gx();
  imu_msg.angular_velocity.y = imu.gy();
  imu_msg.angular_velocity.z = imu.gz();

  imu_msg.linear_acceleration.x = imu.ax();
  imu_msg.linear_acceleration.y = imu.ay();
  imu_msg.linear_acceleration.z = imu.az();

  // Publish IMU data
  RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

  // Spin executor
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}
