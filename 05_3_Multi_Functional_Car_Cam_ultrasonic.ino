/*
  Project: Low-Cost Embodied AI Robot for Indoor Surveillance

  This firmware is based on the example code provided with the Freenove 4WD Smart Car Kit.
  The original Freenove code was used as the hardware-control foundation for camera streaming,
  motor control, servo control, and sensor access.

  Modifications made for this project include:
  - additional HTTP control commands for autonomous patrol behaviour
  - IMU/MPU6050 integration for yaw estimation
  - support for yaw-based turning and heading alignment
  - additional state reporting endpoints
  - changes to support Python-based external control and computer vision processing

  Original hardware platform: Freenove 4WD Smart Car Kit
  Modified by: Sankaran Iyer
  Purpose: Research and educational demonstration of low-cost embodied AI for indoor surveillance.

  Note:
  This file should be used in accordance with the licence terms of the original Freenove code
  and any third-party libraries used in this project.
*/
/**********************************************************************
  Filename    : IR_Receiver_Car.ino
  Product     : Freenove 4WD Car for ESP32
  Auther      : www.freenove.com
  Modification: 2024/08/12
**********************************************************************/

#include <Arduino.h>
#include "Freenove_IR_Lib_for_ESP32.h"
#include "Freenove_4WD_Car_For_ESP32.h"
//#include "Freenove_4WD_Car_Emotion.h"
#include "Freenove_WS2812_Lib_for_ESP32.h"
#include "camera_stream.h"
#include "I2Cdev.h"
#include "MPU6050.h"

//Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(12, 32, 0, TYPE_GRB);
//byte m_color[5][3] = { {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}, {0, 0, 0} };


#define RECV_PIN     0        // Infrared receiving pin
Freenove_ESP32_IR_Recv ir_recv(RECV_PIN);  // Create a class object used to receive class

static int servo_1_angle=90;
static int servo_2_angle=90;
//static int emotion_flag=0;
static int ws2812_flag=0;
long g_distanceCm = -1;
MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float gz_offset = 0.0;

float turn_yaw_deg = 0.0;     // reset for every turn command
float global_yaw_deg = 0.0;   // continuous heading from doorway/start

unsigned long last_imu_time = 0;
bool imu_ok = false;
bool g_forwardActive = false;
unsigned long g_lastSafetyCheckMs = 0;
const long BLOCK_DISTANCE_CM = 40;
//#define TRIG_PIN 14
//#define ECHO_PIN 15
/*
long readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000UL);

  if (duration == 0) return -1;

  return duration * 0.0343 / 2.0;
}
*/
void calibrateGyroZ()
{
  long sum = 0;
  const int N = 200;

  Serial.println("Keep robot still. Calibrating gyro Z...");

  for (int i = 0; i < N; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(10);
  }

  gz_offset = sum / (float)N;

  Serial.print("gz_offset = ");
  Serial.println(gz_offset);
}


void resetTurnYaw()
{
  turn_yaw_deg = 0.0;
  last_imu_time = millis();
}


void resetGlobalYaw()
{
  global_yaw_deg = 0.0;
  last_imu_time = millis();
  Serial.println("Global yaw reset to 0");
}


void updateYaw()
{
  if (!imu_ok) return;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = millis();
  float dt = (now - last_imu_time) / 1000.0;
  last_imu_time = now;

  float gyro_z_dps = (gz - gz_offset) / 131.0;

  if (abs(gyro_z_dps) < 0.5) {
    gyro_z_dps = 0.0;
  }
  float delta_yaw = gyro_z_dps * dt;

  turn_yaw_deg += delta_yaw;
  global_yaw_deg += delta_yaw;
}


void IMU_Setup()
{
  Serial.println("Initializing MPU6050...");

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    imu_ok = false;
    return;
  }

  Serial.println("MPU6050 connection successful");
  imu_ok = true;

  calibrateGyroZ();

  turn_yaw_deg = 0.0;
  global_yaw_deg = 0.0;
  last_imu_time = millis();
}

String GetRobotStateJson()
{
  updateYaw();

  g_distanceCm = (long)Get_Sonar();

  String json = "{";
  json += "\"yaw\":" + String(global_yaw_deg, 2);
  json += ",\"turn_yaw\":" + String(turn_yaw_deg, 2);
  json += ",\"pan\":" + String(servo_1_angle);
  json += ",\"tilt\":" + String(servo_2_angle);
  json += ",\"distance_cm\":" + String(g_distanceCm);
  json += "}";

  return json;
}

void Turn_By_Angle(float target_deg, bool turnRight)
{
  if (!imu_ok) {
    Serial.println("IMU not available; cannot angle-turn");
    Motor_Move(0, 0, 0, 0);
    return;
  }

  int fast_speed_R = 1000;
  int slow_speed_R = 450;

  int fast_speed_L = 1050;
  int slow_speed_L = 380;

  float tolerance = 3.0;
  float threshold_fast = target_deg * 0.80;

  resetTurnYaw();

  Serial.print("IMU turn ");
  Serial.print(turnRight ? "right " : "left ");
  Serial.println(target_deg);

  // Phase 1: fast turn
  if (turnRight) {
    Motor_Move(fast_speed_R, fast_speed_R, -fast_speed_R, -fast_speed_R);
  } else {
    Motor_Move(-fast_speed_L, -fast_speed_L, fast_speed_L, fast_speed_L);
  }

  unsigned long startMs = millis();

  while (abs(turn_yaw_deg) < threshold_fast) {
    updateYaw();

    if (millis() - startMs > 4000) {
      Serial.println("IMU fast turn timeout");
      break;
    }

    delay(5);
  }

  // Phase 2: slow correction
  if (turnRight) {
    Motor_Move(slow_speed_R, slow_speed_R, -slow_speed_R, -slow_speed_R);
  } else {
    Motor_Move(-slow_speed_L, -slow_speed_L, slow_speed_L, slow_speed_L);
  }

  startMs = millis();

  while (abs(turn_yaw_deg) < target_deg - tolerance) {
    updateYaw();

    if (millis() - startMs > 4000) {
      Serial.println("IMU slow correction timeout");
      break;
    }

    delay(5);
  }

  Motor_Move(0, 0, 0, 0);
  g_forwardActive = false;

  delay(100);

  updateYaw();

  Serial.print("Final turn_yaw_deg = ");
  Serial.println(turn_yaw_deg);

  Serial.print("Global yaw_deg = ");
  Serial.println(global_yaw_deg);
}

void setup()
{
  Serial.begin(115200);
  Motor_Move(0, 0, 0, 0);
  PCA9685_Setup();

  IMU_Setup();

  Buzzer_Setup();
  Ultrasonic_Setup();
  Servo_1_Angle(servo_1_angle);
  Servo_2_Angle(servo_2_angle);
  CameraStream_Init();
}
void loop()
{
  ir_recv.task();
  if (ir_recv.nec_available()) {
    unsigned long value = ir_recv.data();
    handleControl(value);
    Serial.print(value, HEX);
    Serial.println();
  }

  if (g_forwardActive)
  {
    long frontDistance = ReadFrontDistanceCm();

    Serial.print("Safety frontDistance = ");
    Serial.println(frontDistance);

    if (frontDistance > 0 && frontDistance < BLOCK_DISTANCE_CM) {
        Serial.println("Blocked while moving");
        Motor_Move(0, 0, 0, 0);
        g_forwardActive = false;
    }
  } 
}
void handleControl(unsigned long value) 
{
  // Handle the commands
  int pos_speed = 1000;
  int neg_speed = -1000;
  switch (value) {
    case 0xFF02FD:// Receive the number '+'
      Motor_Move(pos_speed,pos_speed,pos_speed,pos_speed);
      break;
    case 0xFF9867:// Receive the number '-'
      Motor_Move(neg_speed,neg_speed,neg_speed,neg_speed);
      break;
    case 0xFFE01F:// Receive the number '|<<'
      Motor_Move(neg_speed,neg_speed,pos_speed,pos_speed);
      delay(200);
      Motor_Move(0,0,0,0);
      break;
    case 0xFF906F:// Receive the number '>>|'
      Motor_Move(pos_speed,pos_speed,neg_speed,neg_speed);
      delay(200);
      Motor_Move(0,0,0,0);      
      break;
    case 0xFFA857:// Receive the number '▶'
      Motor_Move(0,0,0,0);
      break;
    case 0xFF6897:// Receive the number '0'
      servo_1_angle=servo_1_angle+10;
      Servo_1_Angle(servo_1_angle);
      break;
    case 0xFF30CF:// Receive the number '1'
      servo_1_angle=servo_1_angle-10;
      Servo_1_Angle(servo_1_angle);
      break;   
    case 0xFF10EF:// Receive the number '4'
      servo_1_angle=90;
      Servo_1_Angle(servo_1_angle);
      break; 
    case 0xFFB04F:// Receive the number 'C'
      servo_2_angle=servo_2_angle+10;
      Servo_2_Angle(servo_2_angle);
      break;      
    case 0xFF7A85:// Receive the number '3'
      servo_2_angle=servo_2_angle-10;
      Servo_2_Angle(servo_2_angle);
      break;
    case 0xFF5AA5:// Receive the number '6'
      servo_2_angle=90;
      Servo_2_Angle(servo_2_angle);
      break;      
    case 0xFF22DD:// Receive the number 'TEST'
      Buzzer_Alert(1,1);
      break;
    /*
    case 0xFF18E7:// Receive the number '2'
      int new_emotion;
      do {
          new_emotion = random(21);
      } while (new_emotion == emotion_flag);
      emotion_flag = new_emotion;
      staticEmtions(emotion_flag);
      break;
    case 0xFF38C7:// Receive the number '5'
      clearEmtions();
      break;
    
    case 0xFF42BD:// Receive the number '7'
      int new_ws2812;
      do {
          new_ws2812 = random(4);
      } while (new_ws2812 == ws2812_flag);
      ws2812_flag = new_ws2812;
      WS2812_Show();
      break;
    case 0xFF4AB5:// Receive the number '8'
      ws2812_flag=4;
      WS2812_Show();
      break; 
    case 0xFF52AD:// Receive the number '9'
      break;  
    case 0xFFFFFFFF:// Remain unchanged
      break;*/
    default:
      break;
  }
}

long ReadCurrentDistanceCm()
{
  delay(40);  // reduce from 80 if servo has not just moved

  float d1 = Get_Sonar();
  delay(25);
  float d2 = Get_Sonar();
  delay(25);
  float d3 = Get_Sonar();

  float d = max(min(d1, d2), min(max(d1, d2), d3));  // median of 3

  g_distanceCm = (long)d;
  return g_distanceCm;
}

long ReadFrontDistanceCm()
{
  Servo_1_Angle(90);
  delay(80);
  return ReadCurrentDistanceCm();
}

void handleControlCmd(String cmd)
{
  int pos_speed = 1000;
  int neg_speed = -1000;
 
  Serial.print("Executing cmd: ");
  Serial.println(cmd);

  // Drive
if (cmd == "forward") {
  Serial.println("FORWARD branch entered");

  long frontDistance = ReadFrontDistanceCm();
  Serial.print("frontDistance = ");
  Serial.println(frontDistance);

  if (frontDistance > 0 && frontDistance < BLOCK_DISTANCE_CM) {
    Serial.println("Blocked: obstacle ahead");
    Motor_Move(0, 0, 0, 0);
    g_forwardActive = false;
    return;
  }

  Motor_Move(pos_speed, pos_speed, pos_speed, pos_speed);
  g_forwardActive = true;
}
else if (cmd == "forward_short") {
  long frontDistance = ReadFrontDistanceCm();
  g_distanceCm = frontDistance;

  if (frontDistance > 0 && frontDistance < 40) {
    Serial.println("Blocked: obstacle ahead");
    Motor_Move(0, 0, 0, 0);
    return;
  }

  Motor_Move(pos_speed, pos_speed, pos_speed, pos_speed);
  delay(300);
  Motor_Move(0, 0, 0, 0);
}

else if (cmd == "forward_medium") {
  long frontDistance = ReadFrontDistanceCm();
  g_distanceCm = frontDistance;

  if (frontDistance > 0 && frontDistance < 40) {
    Serial.println("Blocked: obstacle ahead");
    Motor_Move(0, 0, 0, 0);
    return;
  }

  Motor_Move(pos_speed, pos_speed, pos_speed, pos_speed);
  delay(700);
  Motor_Move(0, 0, 0, 0);
}

else if (cmd == "forward_long") {
  long frontDistance = ReadFrontDistanceCm();
  g_distanceCm = frontDistance;

  if (frontDistance > 0 && frontDistance < 40) {
    Serial.println("Blocked: obstacle ahead");
    Motor_Move(0, 0, 0, 0);
    return;
  }

  Motor_Move(pos_speed, pos_speed, pos_speed, pos_speed);
  delay(1200);
  Motor_Move(0, 0, 0, 0);
}
else if (cmd == "back") {
  Serial.println("BACK branch entered");
  ReadFrontDistanceCm();
  Motor_Move(neg_speed, neg_speed, neg_speed, neg_speed);
  g_forwardActive = false;
}
else if (cmd == "left") {
  ReadFrontDistanceCm();
  Motor_Move(neg_speed, neg_speed, pos_speed, pos_speed);
  g_forwardActive = false;
}
else if (cmd == "right") {
  ReadFrontDistanceCm();
  Motor_Move(pos_speed, pos_speed, neg_speed, neg_speed);
  g_forwardActive = false;
}
else if (cmd == "turn_left_1") {
  Turn_By_Angle(1, false);
}
else if (cmd == "turn_right_1") {
  Turn_By_Angle(1, true);
}
else if (cmd == "turn_left_5") {
  Turn_By_Angle(5, false);
}
else if (cmd == "turn_right_5") {
  Turn_By_Angle(5, true);
}
else if (cmd == "turn_left_10") {
  Turn_By_Angle(10, false);
}
else if (cmd == "turn_right_10") {
  Turn_By_Angle(10, true);
}
else if (cmd == "turn_left_15") {
  Turn_By_Angle(15, false);
}
else if (cmd == "turn_right_15") {
  Turn_By_Angle(15, true);
}
else if (cmd == "turn_left_30") {
  Turn_By_Angle(30, false);
}
else if (cmd == "turn_right_30") {
  Turn_By_Angle(30, true);
}
else if (cmd == "turn_left_90") {
  Turn_By_Angle(90, false);
}
else if (cmd == "turn_right_90") {
  Turn_By_Angle(90, true);
}
else if (cmd == "turn_around") {
  Turn_By_Angle(180, true);
}
else if (cmd == "stop") {
  Motor_Move(0, 0, 0, 0);
  g_forwardActive = false;
}

// Servo 1 = pan (left/right reversed as requested)
else if (cmd == "pan_left") {
  Serial.println("PAN LEFT branch entered");
  servo_1_angle = constrain(servo_1_angle + 10, 60, 150);
  Servo_1_Angle(servo_1_angle);
  ReadCurrentDistanceCm();
}
else if (cmd == "pan_right") {
  servo_1_angle = constrain(servo_1_angle - 10, 60, 150);
  Servo_1_Angle(servo_1_angle);
  ReadCurrentDistanceCm();
}
else if (cmd == "pan_center") {
  servo_1_angle = 90;
  Servo_1_Angle(servo_1_angle);
  ReadCurrentDistanceCm();
}
// Servo 2 = tilt
else if (cmd == "tilt_up") {
  Serial.println("TILT UP branch entered");
  servo_2_angle = constrain(servo_2_angle + 10, 60, 150);
  Servo_2_Angle(servo_2_angle);
  ReadCurrentDistanceCm();
}
else if (cmd == "tilt_down") {
  servo_2_angle = constrain(servo_2_angle - 10, 60, 150);
  Servo_2_Angle(servo_2_angle);
  ReadCurrentDistanceCm();
}
else if (cmd == "tilt_center") {
  servo_2_angle = 90;
  Servo_2_Angle(servo_2_angle);
  ReadCurrentDistanceCm();
}

// Buzzer
else if (cmd == "buzzer") {
  Buzzer_Alert(1, 1);
}
else if (cmd == "imu_status") {
  updateYaw();
  Serial.print("Global yaw_deg = ");
  Serial.println(global_yaw_deg);
}
else if (cmd == "measure_distance") {
  Serial.println("MEASURE DISTANCE branch entered");
  g_distanceCm = ReadFrontDistanceCm();
  Serial.print("Measured front distance = ");
  Serial.println(g_distanceCm);
}
else if (cmd == "reset_global_yaw") {
  resetGlobalYaw();
}
// Emotions
/*
else if (cmd == "emotion_random") {
  int new_emotion;
  do {
    new_emotion = random(21);
  } while (new_emotion == emotion_flag);
  emotion_flag = new_emotion;
  staticEmtions(emotion_flag);
}
else if (cmd == "emotion_clear") {
  clearEmtions();
}*/

// Lights
/*
else if (cmd == "light_random") {
  int new_ws2812;
  do {
    new_ws2812 = random(4);
  } while (new_ws2812 == ws2812_flag);
  ws2812_flag = new_ws2812;
  WS2812_Show();
}
else if (cmd == "light_off") {
  ws2812_flag = 4;
  WS2812_Show();
}*/
}
/*
void WS2812_Show()
{
  for (int i = 0; i < 12; i++)
    strip.setLedColorData(i, m_color[ws2812_flag][0], m_color[ws2812_flag][1], m_color[ws2812_flag][2]);
    strip.show();
}*/