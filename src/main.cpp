#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <PID_v1.h>
#include <Ewma.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/color_rgba.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <LDS_RPLIDAR_A1.h>
#include <rosidl_runtime_c/string_functions.h>
#include <cstring>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <IPAddress.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>

/*==========================================
=               ROS                        =
==========================================*/
IPAddress agent_ip(192, 168, 0, 155);
size_t agent_port = 8888;

char ssid[] = "";
char psk[] = "";

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist twis_msg;

rcl_subscription_t left_led_sub;
std_msgs__msg__ColorRGBA left_led_msg;

rcl_subscription_t right_led_sub;
std_msgs__msg__ColorRGBA right_led_msg;

rcl_subscription_t servoA_sub;
std_msgs__msg__Int16 servoA_msg;

rcl_subscription_t servoB_sub;
std_msgs__msg__Int16 servoB_msg;

rcl_publisher_t sonar_pub;
sensor_msgs__msg__Range sonar_msg;

sensor_msgs__msg__LaserScan scan_msg;
/*==========================================
=               Declarations               =
==========================================*/
#define led_pin 23
#define led_count 2

#define servoA_pin 18
#define servoB_pin 19

#define right_encoder_pin1 39
#define right_encoder_pin2 36

#define left_encoder_pin1 35
#define left_encoder_pin2 34

#define right_motor_control_pin1 25
#define right_motor_control_pin2 33
#define right_motor_speed_pin 32

#define left_motor_control_pin1 27
#define left_motor_control_pin2 26
#define left_motor_speed_pin 13

const int motor_pwm_frequency = 35000;
const byte motor_pwm_resolution = 8;

const int min_motor_pwm = 0;

const float encoder_ticks_per_mtr = 1741;
volatile long right_encoder_ticks = 0;
volatile long left_encoder_ticks = 0;
volatile long left_last_encoder_ticks = 0;
volatile long right_last_encoder_ticks = 0;
volatile long right_encoder_diff = 0;
volatile long left_encoder_diff = 0;

// robot defnitions
double wheel_radius = 0.0335;
double wheel_distance = 0.165;

// Define the timer interrupt frequency (100 ms)
#define ENCODER_TIMER_INTERVAL_US 100000
hw_timer_t *encoder_timer = NULL;

double linear_x = 0.0;
double angular_z = 0.0;

float left_demand_speed = 0.0;
float right_demand_speed = 0.0;

// PID Delarations
double right_kp = 2.3, right_ki = 2, right_kd = 1;
double right_speed_actual = 0, right_speed_output = 0, right_speed_setpoint = 0;
PID right_wheel_pid(&right_speed_actual, &right_speed_output, &right_speed_setpoint, right_kp, right_ki, right_kd, DIRECT);

double left_kp = 2.3, left_ki = 2, left_kd = 1;
double left_speed_actual = 0, left_speed_output = 0, left_speed_setpoint = 0;
PID left_wheel_pid(&left_speed_actual, &left_speed_output, &left_speed_setpoint, left_kp, left_ki, left_kd, DIRECT);

Ewma left_encoder_filter(0.08);
Ewma right_encoder_filter(0.08);

Adafruit_NeoPixel leds(led_count, led_pin, NEO_GRB + NEO_KHZ800);

Servo servoA;
Servo servoB;

enum SonarState
{
  START_MEASUREMENT,
  WAIT_MEASUREMENT,
  READ_DISTANCE
};

const int sonarAddress = 0x57;
bool isSonarAvailable = false;

SonarState sonar_current_state = START_MEASUREMENT;
unsigned long sonar_start_millis = 0;
unsigned char sonar_bytes[3] = {0, 0, 0};
unsigned int sonar_bytes_index = 0;
unsigned long sonar_distance = 0;
unsigned long sonar_last_publish_interval = millis();
const unsigned long sonar_publish_interval = 1000 / 10;

const uint8_t LDS_MOTOR_PWM_PIN = 4; // LiDAR motor speed control using PWM
#define LDS_MOTOR_PWM_FREQ 30000
#define LDS_MOTOR_PWM_BITS 8
HardwareSerial LidarSerial(2); // TX 17, RX 16
LDS_RPLIDAR_A1 lidar;
bool isLidarAvailable = false;
unsigned long lidar_last_publish_interval = millis();
unsigned long lidar_publish_interval = 1000 / 30;
WiFiUDP udp;
unsigned int lidar_udp_port = 9999;

enum SystemState
{
  STATE_INIT,
  STATE_RUNNING,
  STATE_ERROR,
  STATE_RECOVERY
};

SystemState currentState = STATE_INIT;

// Add this before the function prototypes section
enum ErrorCode
{
  ERROR_ROS_INIT,
  ERROR_SENSOR_INIT,
  ERROR_COMMUNICATION
};

#define SONAR_INIT_RETRIES 3
#define SONAR_INIT_DELAY 500

/*==========================================
=              Function Prototypes         =
==========================================*/
// Encoder and Motor Control
void IRAM_ATTR right_encoder_isr();
void IRAM_ATTR left_encoder_isr();
void IRAM_ATTR encoderTimerISR();
void rotateMotor(int left_speed, int right_speed);
void lidarTask(void *pvParameters);
void mainTask(void *pvParameters);

// ROS Callbacks
void cmd_vel_callback(const void *msgin);
void left_led_callback(const void *left_led_msg);
void right_led_callback(const void *right_led_msg);
void servoA_callback(const void *msg);
void servoB_callback(const void *msg);

// Sensor Functions
void updatePing();

// LiDAR Functions
int lidar_serial_read_callback();
size_t lidar_serial_write_callback(const uint8_t *buffer, size_t length);
void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed);
void lidar_info_callback(LDS::info_t code, String info);
void lidar_error_callback(LDS::result_t code, String aux_info);
void send_lidar_data_udp(const sensor_msgs__msg__LaserScan &scan_msg);

// Resource Management
void cleanup();
void cleanup_resources();

// Error Handling
int error_handler(ErrorCode error);
void error_loop();

// System Management
bool initializeTimer();
void setupWatchdog();
void watchdogCallback();

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

// Error handle loop
void error_loop()
{
  while (1)
  {
    delay(100);
  }
}

/*==========================================
=                   Setup                  =
==========================================*/

void setup()
{
  Serial.begin(115200);
  // Encoder Setup
  pinMode(right_encoder_pin1, INPUT);
  pinMode(right_encoder_pin2, INPUT);
  pinMode(left_encoder_pin1, INPUT);
  pinMode(left_encoder_pin2, INPUT);
  delay(100);
  attachInterrupt(left_encoder_pin1, left_encoder_isr, FALLING);
  delay(100);
  attachInterrupt(right_encoder_pin1, right_encoder_isr, FALLING);

  // Initialize timer object
  encoder_timer = timerBegin(0, 80, true);                     // Timer 0, prescaler 80, count up
  timerAttachInterrupt(encoder_timer, &encoderTimerISR, true); // Attach ISR function
  timerAlarmWrite(encoder_timer, ENCODER_TIMER_INTERVAL_US, true);
  timerAlarmEnable(encoder_timer);
  timerStart(encoder_timer);

  right_wheel_pid.SetMode(AUTOMATIC);
  right_wheel_pid.SetOutputLimits(-255, 255);

  left_wheel_pid.SetMode(AUTOMATIC);
  left_wheel_pid.SetOutputLimits(-255, 255);

  // Motors Setup
  pinMode(right_motor_control_pin1, OUTPUT);
  pinMode(right_motor_control_pin2, OUTPUT);
  pinMode(right_motor_speed_pin, OUTPUT);

  pinMode(left_motor_control_pin1, OUTPUT);
  pinMode(left_motor_control_pin2, OUTPUT);
  pinMode(left_motor_speed_pin, OUTPUT);

  leds.begin();
  ESP32PWM::allocateTimer(3);
  servoA.setPeriodHertz(50);
  servoA.attach(servoA_pin, 550, 2500);
  servoB.setPeriodHertz(50);
  servoB.attach(servoB_pin, 550, 2500);
  delay(500);

  Wire.begin();
  isSonarAvailable = false;

  // Try multiple times to initialize the sonar
  for (int retry = 0; retry < SONAR_INIT_RETRIES; retry++)
  {
    delay(SONAR_INIT_DELAY); // Give more time between attempts

    Wire.beginTransmission(sonarAddress);
    byte error = Wire.endTransmission();

    if (error == 0)
    {
      isSonarAvailable = true;
      sonar_msg.range = 0;
      Serial.println("Sonar sensor initialized successfully");
      break;
    }
    else
    {
      Serial.printf("Sonar init attempt %d failed with error: %d\n", retry + 1, error);

      // Reset I2C bus if we're going to retry
      if (retry < SONAR_INIT_RETRIES - 1)
      {
        Wire.end();
        delay(100);
        Wire.begin();
      }
    }
  }

  if (!isSonarAvailable)
  {
    Serial.println("Failed to initialize sonar sensor after all retries");
  }

  LidarSerial.setRxBufferSize(1024);
  uint32_t baud_rate = lidar.getSerialBaudRate();
  LidarSerial.begin(baud_rate, SERIAL_8N1, 16, 17);
  lidar.setScanPointCallback(lidar_scan_point_callback);
  lidar.setSerialWriteCallback(lidar_serial_write_callback);
  lidar.setSerialReadCallback(lidar_serial_read_callback);
  analogWrite(LDS_MOTOR_PWM_PIN, 255);
  delay(3000);
  lidar.init();

  LDS::result_t result = lidar.start();
  if (result == LDS::RESULT_OK)
  {
    Serial.println("LiDAR started successfully.");
    isLidarAvailable = true;
    rosidl_runtime_c__String__init(&scan_msg.header.frame_id);
    rosidl_runtime_c__String__assign(&scan_msg.header.frame_id, "scan");
    scan_msg.angle_min = 0.0;                        // Min angle in radians
    scan_msg.angle_max = 2.0 * M_PI;                 // Full circle in radians
    scan_msg.angle_increment = (2.0 * M_PI) / 360.0; // Example: 1 degree per reading in radians
    scan_msg.range_min = 0.15;                       // Min range of the lidar in meters
    scan_msg.range_max = 6.0;                        // Max range of the lidar in meters
    scan_msg.ranges.size = 360;
    scan_msg.ranges.data = (float *)calloc(360, sizeof(float));
    scan_msg.ranges.capacity = 360;
    std::fill_n(scan_msg.ranges.data, 360, std::numeric_limits<float>::infinity());

    scan_msg.intensities.size = 0;
    scan_msg.intensities.data = (float *)calloc(0, sizeof(float));
    scan_msg.intensities.capacity = 0;
  }
  else
  {
    isLidarAvailable = false;
    Serial.print("LiDAR failed to start. Error: ");
    Serial.println(lidar.resultCodeToString(result));
  }
  // ros
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  delay(2000);
  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "rio", "", &support));

  // Create subscriptions and publishers
  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
      &left_led_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
      "left_led"));

  RCCHECK(rclc_subscription_init_default(
      &right_led_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
      "right_led"));

  RCCHECK(rclc_subscription_init_default(
      &servoA_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "servoA"));

  RCCHECK(rclc_subscription_init_default(
      &servoB_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "servoB"));

  if (isSonarAvailable)
  {
    RCCHECK(rclc_publisher_init_best_effort(
        &sonar_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "sonar"));
  }

  // Initialize the executor with the correct number of handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));

  // Add subscriptions to executor
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &twis_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &left_led_sub, &left_led_msg, &left_led_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &right_led_sub, &right_led_msg, &right_led_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servoA_sub, &servoA_msg, &servoA_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servoB_sub, &servoB_msg, &servoB_callback, ON_NEW_DATA));
  // Create tasks
  xTaskCreatePinnedToCore(
      lidarTask,    // Task function
      "Lidar Task", // Name of the task
      4096,         // Stack size
      NULL,         // Task input parameter
      1,            // Priority of the task
      NULL,         // Task handle
      0             // Core where the task should run
  );

  xTaskCreatePinnedToCore(
      mainTask,    // Task function
      "Main Task", // Name of the task
      4096,        // Stack size
      NULL,        // Task input parameter
      1,           // Priority of the task
      NULL,        // Task handle
      1            // Core where the task should run
  );

  // Initialize UDP
  udp.begin(lidar_udp_port);
}

/*==========================================
=                   Loop                   =
==========================================*/

void loop()
{
}
void lidarTask(void *pvParameters)
{
  while (true)
  {

    if (isLidarAvailable)
    {
      lidar.loop();

      if (millis() - lidar_last_publish_interval > lidar_publish_interval)
      {
        lidar_last_publish_interval = millis();
        send_lidar_data_udp(scan_msg);
      }
    }
    vTaskDelay(1); // Yield to other tasks
  }
}

void mainTask(void *pvParameters)
{
  while (true)
  {
    // Only use the executor spin
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

    if (isSonarAvailable)
    {
      updatePing();

      if (millis() - sonar_last_publish_interval > sonar_publish_interval)
      {
        sonar_last_publish_interval = millis();
        sonar_msg.min_range = 0.02;
        sonar_msg.max_range = 4.0;
        rosidl_runtime_c__String__init(&sonar_msg.header.frame_id);
        rosidl_runtime_c__String__assign(&sonar_msg.header.frame_id, "sonar");
        struct timespec tv = {0};
        clock_gettime(0, &tv);
        sonar_msg.header.stamp.nanosec = tv.tv_nsec;
        sonar_msg.header.stamp.sec = tv.tv_sec;
        rcl_ret_t ret = rcl_publish(&sonar_pub, &sonar_msg, NULL);
        if (ret != RCL_RET_OK)
        {
          error_handler(ERROR_COMMUNICATION);
        }
      }
    }

    vTaskDelay(1); // Yield to other tasks
  }
}

void IRAM_ATTR right_encoder_isr()
{
  // Read the value for the encoder for the right wheel
  if (digitalRead(right_encoder_pin2) == LOW)
  {
    right_encoder_ticks++;
  }
  else
  {
    right_encoder_ticks--;
  }
}

void IRAM_ATTR left_encoder_isr()
{
  // Read the value for the encoder for the left wheel
  if (digitalRead(left_encoder_pin2) == LOW)
  {
    left_encoder_ticks--;
  }
  else
  {
    left_encoder_ticks++;
  }
}
void IRAM_ATTR encoderTimerISR()
{
  left_encoder_diff = left_encoder_ticks - left_last_encoder_ticks;
  left_last_encoder_ticks = left_encoder_ticks;

  right_encoder_diff = right_encoder_ticks - right_last_encoder_ticks;
  right_last_encoder_ticks = right_encoder_ticks;

  // encoder ticks required for 1m/s 1741
  // per millisecond 1741/1000 = 1.741tick
  // per 100ms measuring interval = 1.741*100
  left_speed_actual = left_encoder_filter.filter(left_encoder_diff);
  left_speed_setpoint = left_demand_speed * 174.1;
  left_wheel_pid.Compute();

  right_speed_actual = right_encoder_filter.filter(right_encoder_diff);
  right_speed_setpoint = right_demand_speed * 174.1;
  right_wheel_pid.Compute();

  rotateMotor(left_speed_output, right_speed_output);
}
// rotate motor
void rotateMotor(int left_speed, int right_speed)
{
  // left motor
  if (left_speed > -12 && left_speed < 12)
    left_speed = 0;
  analogWrite(left_motor_speed_pin, abs(left_speed));
  if (left_speed == 0)
  {
    digitalWrite(left_motor_control_pin1, LOW);
    digitalWrite(left_motor_control_pin2, LOW);
  }
  else if (left_speed > 0)
  {
    digitalWrite(left_motor_control_pin1, HIGH);
    digitalWrite(left_motor_control_pin2, LOW);
  }
  else if (left_speed < 0)
  {
    digitalWrite(left_motor_control_pin1, LOW);
    digitalWrite(left_motor_control_pin2, HIGH);
  }

  // right motor
  if (right_speed > -12 && right_speed < 12)
    right_speed = 0;
  analogWrite(right_motor_speed_pin, abs(right_speed));
  if (right_speed == 0)
  {
    digitalWrite(right_motor_control_pin1, LOW);
    digitalWrite(right_motor_control_pin2, LOW);
  }
  else if (right_speed > 0)
  {
    digitalWrite(right_motor_control_pin1, LOW);
    digitalWrite(right_motor_control_pin2, HIGH);
  }
  else if (right_speed < 0)
  {
    digitalWrite(right_motor_control_pin1, HIGH);
    digitalWrite(right_motor_control_pin2, LOW);
  }
}

void cmd_vel_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg_twist = (const geometry_msgs__msg__Twist *)msgin;
  linear_x = msg_twist->linear.x;
  angular_z = msg_twist->angular.z;
  left_demand_speed = linear_x - angular_z * (wheel_distance / 2.0);
  right_demand_speed = linear_x + angular_z * (wheel_distance / 2.0);
}

void left_led_callback(const void *left_led_msg)
{
  const std_msgs__msg__ColorRGBA *msg_led = (const std_msgs__msg__ColorRGBA *)left_led_msg;
  if (msg_led->a <= 255 && msg_led->a >= 0)
    leds.setBrightness(msg_led->a);
  if ((msg_led->r >= 0 && msg_led->r <= 255) && (msg_led->g >= 0 && msg_led->g <= 255) && (msg_led->b >= 0 && msg_led->b <= 255))
    leds.setPixelColor(1, leds.Color(msg_led->r, msg_led->g, msg_led->b));
  leds.show();
}
void right_led_callback(const void *right_led_msg)
{
  const std_msgs__msg__ColorRGBA *msg_led = (const std_msgs__msg__ColorRGBA *)right_led_msg;
  if (msg_led->a <= 255 && msg_led->a >= 0)
    leds.setBrightness(msg_led->a);
  if ((msg_led->r >= 0 && msg_led->r <= 255) && (msg_led->g >= 0 && msg_led->g <= 255) && (msg_led->b >= 0 && msg_led->b <= 255))
    leds.setPixelColor(0, leds.Color(msg_led->r, msg_led->g, msg_led->b));
  leds.show();
}
void servoA_callback(const void *msg)
{
  const std_msgs__msg__Int16 *servo_msg = (const std_msgs__msg__Int16 *)msg;
  if (servo_msg->data <= 180 && servo_msg->data >= 0)
  {
    servoA.write(servo_msg->data);
  }
}
void servoB_callback(const void *msg)
{
  const std_msgs__msg__Int16 *servo_msg = (const std_msgs__msg__Int16 *)msg;
  if (servo_msg->data <= 180 && servo_msg->data >= 0)
  {
    servoB.write(servo_msg->data);
  }
}
void updatePing()
{

  switch (sonar_current_state)
  {
  case START_MEASUREMENT:
    Wire.beginTransmission(sonarAddress);
    Wire.write(1); // 1 = cmd to start measurement
    Wire.endTransmission();
    sonar_start_millis = millis();
    sonar_current_state = WAIT_MEASUREMENT;
    break;

  case WAIT_MEASUREMENT:
    if (millis() - sonar_start_millis >= 120)
    { // 1 cycle approx. 100ms
      sonar_current_state = READ_DISTANCE;
    }
    break;

  case READ_DISTANCE:
    sonar_bytes_index = 0;
    Wire.requestFrom(sonarAddress, 3); // read distance
    while (Wire.available() && sonar_bytes_index < 3)
    {
      sonar_bytes[sonar_bytes_index++] = Wire.read();
    }

    if (sonar_bytes_index == 3)
    {
      sonar_distance = (unsigned long)(sonar_bytes[0]) * 65536;
      sonar_distance = sonar_distance + (unsigned long)(sonar_bytes[1]) * 256;
      sonar_distance = (sonar_distance + (unsigned long)(sonar_bytes[2])) / 10000;
    }
    else
    {
      sonar_distance = 0; // Error in reading
    }

    if ((1 <= sonar_distance) && (900 >= sonar_distance))
    { // measured value between 1cm to 9 meters
      sonar_msg.range = (float)sonar_distance;
    }
    else
    {
      sonar_msg.range = 0.0;
    }
    sonar_current_state = START_MEASUREMENT;
    break;
  }
}

int lidar_serial_read_callback()
{
  int c = LidarSerial.read();
  return c;
}

size_t lidar_serial_write_callback(const uint8_t *buffer, size_t length)
{
  return LidarSerial.write(buffer, length);
}

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed)
{
  scan_msg.scan_time = 1.0 / lidar.getCurrentScanFreqHz();
  scan_msg.time_increment = scan_msg.scan_time / 360.0; // Time per point

  // Calculate index for the ranges array
  int index = round(angle_deg);

  // Ensure index is within bounds
  if (index >= 0 && index < scan_msg.ranges.size)
  {
    float distance_m = distance_mm / 1000.0;
    if (distance_m >= scan_msg.range_min && distance_m <= scan_msg.range_max)
    {
      scan_msg.ranges.data[index] = round(distance_m * 100.0) / 100.0;
    }
    else
    {
      scan_msg.ranges.data[index] = std::numeric_limits<float>::infinity();
    }
  }
  int inf_count = 0;
  int start_index = -1;

  for (int i = 0; i < scan_msg.ranges.size; i++)
  {
    if (scan_msg.ranges.data[i] == std::numeric_limits<float>::infinity())
    {
      if (start_index == -1)
      {
        start_index = i;
      }
      inf_count++;
    }
    else
    {
      if (inf_count > 0 && inf_count <= 3)
      {
        // We have a sequence of 1 to 3 'inf' values
        float prev_value = (start_index > 0) ? scan_msg.ranges.data[start_index - 1] : 0.0;
        float next_value = scan_msg.ranges.data[i];

        if (prev_value != std::numeric_limits<float>::infinity() && next_value != std::numeric_limits<float>::infinity())
        {
          // Replace 'inf' values with the average of surrounding decimal values
          float average_value = (prev_value + next_value) / 2.0;
          for (int j = start_index; j < i; j++)
          {
            scan_msg.ranges.data[j] = average_value;
          }
        }
      }
      inf_count = 0;
      start_index = -1;
    }
  }
}
void lidar_info_callback(LDS::info_t code, String info)
{
  Serial.print("LDS info ");
  Serial.print(lidar.infoCodeToString(code));
  Serial.print(": ");
  Serial.println(info);
}

void lidar_error_callback(LDS::result_t code, String aux_info)
{
  Serial.print("LDS error ");
  Serial.print(lidar.resultCodeToString(code));
  Serial.print(": ");
  Serial.println(aux_info);
}

// Add cleanup function
void cleanup()
{
  if (scan_msg.ranges.data)
  {
    free(scan_msg.ranges.data);
    scan_msg.ranges.data = NULL;
  }
  if (scan_msg.intensities.data)
  {
    free(scan_msg.intensities.data);
    scan_msg.intensities.data = NULL;
  }
}

// Call in destructor or error handling

int error_handler(ErrorCode error)
{
  switch (error)
  {
  case ERROR_ROS_INIT:
    // Serial.println("ROS initialization error");
    currentState = STATE_ERROR;
    return -1;
  case ERROR_SENSOR_INIT:
    // Serial.println("Sensor initialization error");
    currentState = STATE_ERROR;
    return -2;
  case ERROR_COMMUNICATION:
    // Serial.println("Communication error");
    currentState = STATE_ERROR;
    return -3;
  default:
    // Serial.println("Unknown error");
    currentState = STATE_ERROR;
    return -99;
  }
}

void cleanup_resources()
{
  // Free memory
  cleanup();

  // Disable interrupts
  timerEnd(encoder_timer);

  // Close communications
  Wire.end();
  LidarSerial.end();
}

void cleanup_ros_resources()
{
  rcl_publisher_fini(&sonar_pub, &node);
  rcl_subscription_fini(&cmd_vel_sub, &node);
  rcl_subscription_fini(&left_led_sub, &node);
  rcl_subscription_fini(&right_led_sub, &node);
  rcl_subscription_fini(&servoA_sub, &node);
  rcl_subscription_fini(&servoB_sub, &node);
  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
}

void send_lidar_data_udp(const sensor_msgs__msg__LaserScan &scan_msg)
{
  // Create a JSON document
  StaticJsonDocument<2048> doc; // Adjust size as needed

  // Add scan frequency
  doc["scan_frequency"] = lidar.getCurrentScanFreqHz();

  // Add distances directly to the JSON document
  JsonArray distances = doc.createNestedArray("distances");
  for (int i = 0; i < scan_msg.ranges.size; i++)
  {
    distances.add(scan_msg.ranges.data[i]);
  }

  // Serialize JSON document to string
  String jsonString;
  serializeJson(doc, jsonString);
  // Send UDP packet
  udp.beginPacket(agent_ip, lidar_udp_port);
  udp.print(jsonString);
  udp.endPacket();
}