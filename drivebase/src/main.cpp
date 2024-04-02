#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#include <sensor_msgs/msg/imu.h>
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Indicate program running properly
#define LED_PIN 2

// Motor control pin definitions
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

// IMU
MPU6050 mpu;

// Physical constants
#define EARTH_GRAVITY_MS2 9.80665  // m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gg;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorInt16 ggWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ROS publisher and message
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

/**
 * Translate integer values to chassis movement command
 *
 * @param command: integer command of desired action
 * @param speed: PWM speed value (0-255)
 * @return void
 */
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
    analogWrite(PIN_ENA_RIGHT_F, 255);

    digitalWrite(PIN_IN5, LOW);
    digitalWrite(PIN_IN6, HIGH);
    digitalWrite(PIN_IN7, LOW);
    digitalWrite(PIN_IN8, HIGH);
    analogWrite(PIN_ENA_LEFT_B, 0);
    analogWrite(PIN_ENA_RIGHT_B, 255);
  } else if (command == RIGHT) { // right
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENA_LEFT_F, 255);
    analogWrite(PIN_ENA_RIGHT_F, 0);

    digitalWrite(PIN_IN5, HIGH);
    digitalWrite(PIN_IN6, LOW);
    digitalWrite(PIN_IN7, HIGH);
    digitalWrite(PIN_IN8, LOW);
    analogWrite(PIN_ENA_LEFT_B, 255);
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

/**
 * Callback function for subscriber
 * 
 * @param msgin: pointer to incoming message
 * @return void
*/
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  // digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
  // 1 - Forward
  // 2 - Backward
  // 3 - Left
  // 4 - Right
  // 5 - Stop
  controlMotors(msg->data, 200);
}


/** Setup function for micro-ROS
 * 
 * @return void
 * */ 
void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

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
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-156);
  mpu.setYGyroOffset(-11);
  mpu.setZGyroOffset(-14);
  mpu.setXAccelOffset(-3699);
  mpu.setYAccelOffset(-2519);
  mpu.setZAccelOffset(1391); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {

      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      mpu.setDMPEnabled(true);

      mpuIntStatus = mpu.getIntStatus();

      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
  }


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
  if (!dmpReady) return;
 
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // Fill orientation data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    imu_msg.orientation.w = q.w;
    imu_msg.orientation.x = q.x;
    imu_msg.orientation.y = q.y;
    imu_msg.orientation.z = q.z;

    // Fill linear acceleration data
    mpu.dmpGetAccel(&aa, fifoBuffer);
    imu_msg.linear_acceleration.x = aa.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    imu_msg.linear_acceleration.y = aa.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    imu_msg.linear_acceleration.z = aa.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;

    // Fill angular velocity data
    mpu.dmpGetGyro(&gg, fifoBuffer);
    imu_msg.angular_velocity.x = gg.x * mpu.get_gyro_resolution() * DEG_TO_RAD;
    imu_msg.angular_velocity.y = gg.y * mpu.get_gyro_resolution() * DEG_TO_RAD;
    imu_msg.angular_velocity.z = gg.z * mpu.get_gyro_resolution() * DEG_TO_RAD;
        
    // Publish IMU data
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

  }

  // Spin executor
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}