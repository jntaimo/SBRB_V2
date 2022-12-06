#include <Arduino.h>
#include "PID.h"

//motor controller pins
#define DIR1 13
#define PWM1 12
#define DIR2 11
#define PWM2 10

// ================================================================
// ===               MOTOR CONTROLLER SETUP               ===
// ================================================================

#define MAX_PWM 255
#define DEADBAND__PWM 0


void motorSetup(){
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
}
// ================================================================
// ===                 Potentiometer Setup                      ===
// ================================================================

//Defines the balancing angle from vertical in degrees
float trim_val = -1.0;//-1.0;
//Adjusts relative power to each motor to make it drive straight
float bal_val = 1.0;

// ================================================================
// ===            TILT ANGLE AND PID VARIABLES                  ===
// ================================================================
float yaw, pitch, pitchRate;
float error, sum_error = 0.0, d_error = 0.0, filt_d_error = 0.0, error_pre;

unsigned long timer;
double loop_time;
float Pcontrol = 0.0, Icontrol = 0.0, Dcontrol = 0.0;
float pwm = 0.0, pwm_L, pwm_R;
float set_point = 0.0;

float kp =13;//5;
float ki =60;//30;
float kd = .2;//0.5;

// ================================================================
// ===      0                INITIAL SETUP                       ===
// ================================================================


//updates all of the relevant controller parameters
void updateController(){
  pitch = pitch - trim_val;
  error = set_point - pitch;
  d_error = (error-error_pre)/0.01;
  error_pre = error;
  sum_error += error*0.01;  
  sum_error = constrain(sum_error, -MAX_PWM/ki, MAX_PWM/ki);
  
}

//Sets the latest PID values after updating the controller
void updatePID(){
  Pcontrol = error * kp;
  Icontrol = sum_error * ki;
  Dcontrol = pitchRate * kd; //use pitch_rate for better performance
  //Icontrol = constrain(Icontrol, -MAX_PWM, MAX_PWM);
  pwm = Pcontrol + Icontrol + Dcontrol;
  
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
}

//Get the setpoint based on the input mode
void getSetpoint(){

}

//sends the correct pwm value to the motors
//assumes the pwm is from -255 to 255
void motorPWM(int pwm_L, int pwm_R){

  pwm_L = constrain(pwm_L, -MAX_PWM, MAX_PWM);
  pwm_R = constrain(pwm_R, -MAX_PWM, MAX_PWM);
    
    //assign motor direction
    digitalWrite(DIR1, pwm_L > 0);
    digitalWrite(DIR2, pwm_R > 0);
    //use deadband for minimum PWM
    if (pwm_L < DEADBAND__PWM && pwm_L >=0) pwm_L = 0;
    if (pwm_L > -DEADBAND__PWM && pwm_L < 0) pwm_L = -0;

    if (pwm_R <  DEADBAND__PWM && pwm_R >=0) pwm_R = 0;
    if (pwm_R > -DEADBAND__PWM && pwm_R < 0) pwm_L = -0;
  
  //TODO Stop motors if tilt angles exceed 15 degrees
  if (abs(pitch) >= 20) {
    pwm_L = 0.0;
    pwm_R = 0.0;
  }

    analogWrite(PWM1, abs(pwm_L));
    analogWrite(PWM2, abs(pwm_R));

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
long printDelay = 120; //milliseconds
long prevPrintTime = 0;

void runPID(float gyroYaw, float gyroPitch, float gyroPitchRate) {
  timer = micros();
  yaw = gyroYaw;
  
  pitch = gyroPitch;
  pitchRate = gyroPitchRate;
  updateController();
  updatePID();

  loop_time = (micros() - timer)/1000000.0 ;//seconds

  if (millis() > prevPrintTime + printDelay){
    prevPrintTime = millis();
    Serial.print("P\t");
    Serial.println(Pcontrol);
    Serial.print("\tI\t");
    Serial.print(Icontrol);
    Serial.print("\tD\t");
    Serial.print(Dcontrol);
    Serial.print("\tSumError\t");
    Serial.println(sum_error);    
  }
    motorPWM(-pwm, pwm);  
}
