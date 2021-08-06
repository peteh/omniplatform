#include <Arduino.h>
#include <Wire.h>
#include "PS2X_lib.h"
#include "QGPMaker_MotorShield.h"
#include <Kinematics.h>
#include <Log.h>
#include <SerialLogger.h>

#define MOTOR_MAX_RPM 107       // motor's maximum rpm
#define WHEEL_DIAMETER 0.065    // robot's wheel diameter expressed in meters
#define FR_WHEEL_DISTANCE 0.15  // distance between front wheel and rear wheel
#define LR_WHEEL_DISTANCE 0.175 // distance between left wheel and right wheel
#define PWM_BITS 8              // microcontroller's PWM pin resolution. Arduino Uno/Mega Teensy is using 8 bits(0-255)

Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, FR_WHEEL_DISTANCE, LR_WHEEL_DISTANCE, PWM_BITS);

QGPMaker_MotorShield AFMS = QGPMaker_MotorShield();
PS2X ps2x;
long ARM_MIN[] = {10, 10, 40, 10};

long ARM_MAX[] = {170, 140, 170, 102};

QGPMaker_Servo *Servo1 = AFMS.getServo(0);
QGPMaker_Servo *Servo2 = AFMS.getServo(1);
QGPMaker_Servo *Servo3 = AFMS.getServo(2);
QGPMaker_Servo *Servo4 = AFMS.getServo(3);
QGPMaker_DCMotor *DCMotor_2 = AFMS.getMotor(2);
QGPMaker_DCMotor *DCMotor_4 = AFMS.getMotor(4);
QGPMaker_DCMotor *DCMotor_1 = AFMS.getMotor(1);
QGPMaker_DCMotor *DCMotor_3 = AFMS.getMotor(3);

void forward()
{
  DCMotor_1->setSpeed(200);
  DCMotor_1->run(FORWARD);
  DCMotor_2->setSpeed(200);
  DCMotor_2->run(FORWARD);
  DCMotor_3->setSpeed(200);
  DCMotor_3->run(FORWARD);
  DCMotor_4->setSpeed(200);
  DCMotor_4->run(FORWARD);
}

void turnLeft()
{
  DCMotor_1->setSpeed(200);
  DCMotor_1->run(BACKWARD);
  DCMotor_2->setSpeed(200);
  DCMotor_2->run(BACKWARD);
  DCMotor_3->setSpeed(200);
  DCMotor_3->run(FORWARD);
  DCMotor_4->setSpeed(200);
  DCMotor_4->run(FORWARD);
}

void turnRight()
{
  DCMotor_1->setSpeed(200);
  DCMotor_1->run(FORWARD);
  DCMotor_2->setSpeed(200);
  DCMotor_2->run(FORWARD);
  DCMotor_3->setSpeed(200);
  DCMotor_3->run(BACKWARD);
  DCMotor_4->setSpeed(200);
  DCMotor_4->run(BACKWARD);
}

void moveLeft()
{
  DCMotor_1->setSpeed(200);
  DCMotor_1->run(BACKWARD);
  DCMotor_2->setSpeed(200);
  DCMotor_2->run(FORWARD);
  DCMotor_3->setSpeed(200);
  DCMotor_3->run(BACKWARD);
  DCMotor_4->setSpeed(200);
  DCMotor_4->run(FORWARD);
}

void moveRight()
{
  DCMotor_1->setSpeed(200);
  DCMotor_1->run(FORWARD);
  DCMotor_2->setSpeed(200);
  DCMotor_2->run(BACKWARD);
  DCMotor_3->setSpeed(200);
  DCMotor_3->run(FORWARD);
  DCMotor_4->setSpeed(200);
  DCMotor_4->run(BACKWARD);
}

void backward()
{
  DCMotor_1->setSpeed(200);
  DCMotor_1->run(BACKWARD);
  DCMotor_2->setSpeed(200);
  DCMotor_2->run(BACKWARD);
  DCMotor_3->setSpeed(200);
  DCMotor_3->run(BACKWARD);
  DCMotor_4->setSpeed(200);
  DCMotor_4->run(BACKWARD);
}

void stopMoving()
{
  DCMotor_1->setSpeed(0);
  DCMotor_1->run(RELEASE);
  DCMotor_2->setSpeed(0);
  DCMotor_2->run(RELEASE);
  DCMotor_3->setSpeed(0);
  DCMotor_3->run(RELEASE);
  DCMotor_4->setSpeed(0);
  DCMotor_4->run(RELEASE);
}

void setup()
{
  Serial.begin(115200);
  AFMS.begin(50);
  Log::init(new SerialLogger());
  int error = 0;
  do
  {
    error = ps2x.config_gamepad(13, 11, 10, 12, true, true);
    if (error == 0)
    {
      break;
    }
    else
    {
      delay(100);
    }
  } while (1);

  Servo1->writeServo(90);
  Servo2->writeServo(90);
  Servo3->writeServo(90);
  Servo4->writeServo(90);
}

void loop()
{
  ps2x.read_gamepad(false, 0);
  delay(30);
  //if (!ps2x.Button(PSB_L1))
  //{
  //  Serial.println("Enable not pressed");
  //  stopMoving();
  //  delay(100);
  //  return;
  //}


  Kinematics::output rpm;
  Kinematics::output pwm;
  //simulated required velocities
  float linear_vel_x = 0.1; // 1 m/s
  float linear_vel_y = 0;   // 0 m/s
  float angular_vel_z = 0;  // 1 rad/s

  //given the required velocities for the robot, you can calculate the rpm required for each motor
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pwm = kinematics.getPWM(linear_vel_x, linear_vel_y, angular_vel_z);

  Serial.print(" FRONT LEFT MOTOR: ");
  // Assuming you have an encoder for each wheel, you can pass this RPM value to a PID controller
  // as a setpoint and your encoder data as a feedback.
  Serial.print(rpm.motor1);
  Serial.print(pwm.motor1);
  // kinematic model M1 = DCMotor1
  DCMotor_1->setSpeed(pwm.motor1);
  DCMotor_1->run(FORWARD);

  Serial.print(" FRONT RIGHT MOTOR: ");
  Serial.print(rpm.motor2);

  Serial.print(" REAR LEFT MOTOR: ");
  Serial.print(rpm.motor3);

  Serial.print(" REAR RIGHT MOTOR: ");
  Serial.println(rpm.motor4);

  delay(5000);

  // This is a simulated feedback from each motor. We'll just pass the calculated rpm above for demo's sake.
  // In a live robot, these should be replaced with real RPM values derived from encoder.
  int motor1_feedback = rpm.motor1; //in rpm
  int motor2_feedback = rpm.motor2; //in rpm
  int motor3_feedback = rpm.motor3; //in rpm
  int motor4_feedback = rpm.motor4; //in rpm

  Kinematics::velocities vel;

  // Now given the RPM from each wheel, you can calculate the linear and angular velocity of the robot.
  // This is useful if you want to create an odometry data (dead reckoning)
  vel = kinematics.getVelocities(motor1_feedback, motor2_feedback, motor3_feedback, motor4_feedback);
  Serial.print(" VEL X: ");
  Serial.print(vel.linear_x, 4);

  Serial.print(" VEL_Y: ");
  Serial.print(vel.linear_y, 4);

  Serial.print(" ANGULAR_Z: ");
  Serial.println(vel.angular_z, 4);
  Serial.println("");
}

void loop2()
{
  ps2x.read_gamepad(false, 0);
  delay(30);
  if (ps2x.Button(PSB_PAD_UP))
  {
    if (ps2x.Button(PSB_L2))
    {
      DCMotor_2->setSpeed(200);
      DCMotor_2->run(FORWARD);
      DCMotor_4->setSpeed(200);
      DCMotor_4->run(FORWARD);
    }
    else if (ps2x.Button(PSB_R2))
    {
      DCMotor_1->setSpeed(200);
      DCMotor_1->run(FORWARD);
      DCMotor_3->setSpeed(200);
      DCMotor_3->run(FORWARD);
    }
    else
    {
      forward();
    }
  }
  else if (ps2x.Button(PSB_PAD_DOWN))
  {
    if (ps2x.Button(PSB_L2))
    {
      DCMotor_2->setSpeed(200);
      DCMotor_2->run(BACKWARD);
      DCMotor_4->setSpeed(200);
      DCMotor_4->run(BACKWARD);
    }
    else if (ps2x.Button(PSB_R2))
    {
      DCMotor_1->setSpeed(200);
      DCMotor_1->run(BACKWARD);
      DCMotor_3->setSpeed(200);
      DCMotor_3->run(BACKWARD);
    }
    else
    {
      backward();
    }
  }
  else if (ps2x.Button(PSB_PAD_LEFT))
  {
    turnLeft();
  }
  else if (ps2x.Button(PSB_PAD_RIGHT))
  {
    turnRight();
  }
  else if (ps2x.Button(PSB_L1))
  {
    moveLeft();
  }
  else if (ps2x.Button(PSB_R1))
  {
    moveRight();
  }
  else
  {
    stopMoving();
  }
  // 按下手柄X按钮，手柄震动一下
  if (ps2x.Button(PSB_CROSS))
  {
    ps2x.read_gamepad(true, 200);
    delay(300);
    ps2x.read_gamepad(false, 0);
  }
  if (ps2x.Analog(PSS_LX) > 240)
  {
    if (Servo1->readDegrees() > ARM_MIN[(int)(0)])
    {
      Servo1->writeServo((Servo1->readDegrees() - 1));
    }
  }
  else if (ps2x.Analog(PSS_LX) < 10)
  {
    if (Servo1->readDegrees() < ARM_MAX[(int)(0)])
    {
      Servo1->writeServo((Servo1->readDegrees() + 1));
    }
  }
  if (ps2x.Analog(PSS_LY) > 240)
  {
    if (Servo2->readDegrees() > ARM_MIN[(int)(1)])
    {
      Servo2->writeServo((Servo2->readDegrees() - 1));
    }
  }
  else if (ps2x.Analog(PSS_LY) < 10)
  {
    if (Servo2->readDegrees() < ARM_MAX[(int)(1)])
    {
      Servo2->writeServo((Servo2->readDegrees() + 1));
    }
  }
  if (ps2x.Analog(PSS_RY) > 240)
  {
    if (Servo3->readDegrees() > ARM_MIN[(int)(2)])
    {
      Servo3->writeServo((Servo3->readDegrees() - 1));
    }
  }
  else if (ps2x.Analog(PSS_RY) < 10)
  {
    if (Servo3->readDegrees() < ARM_MAX[(int)(2)])
    {
      Servo3->writeServo((Servo3->readDegrees() + 1));
    }
  }
  if (ps2x.Analog(PSS_RX) > 240)
  {
    if (Servo4->readDegrees() > ARM_MIN[(int)(3)])
    {
      Servo4->writeServo((Servo4->readDegrees() - 1));
    }
  }
  else if (ps2x.Analog(PSS_RX) < 10)
  {
    if (Servo4->readDegrees() < ARM_MAX[(int)(3)])
    {
      Servo4->writeServo((Servo4->readDegrees() + 1));
    }
  }
  delay(2);
}
