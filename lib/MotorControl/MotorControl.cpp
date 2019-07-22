#include "MotorControl.h"
#include "math.h"
#include <Arduino.h>
#include <WEMOS_Motor.h> 

#define M1_Adress (0x32) 
#define M2_Adress (0x30)
#define M3_Adress (0x2E)  
#define M4_Adress (0x2F)
#define PWM 30
#define PWMR 22

Motor M1(M1_Adress, _MOTOR_B, 1000);
Motor M2(M2_Adress, _MOTOR_B, 1000);
Motor M3(M3_Adress, _MOTOR_B, 1000);
Motor M4(M4_Adress, _MOTOR_B, 1000);

void MotorControl::setupMotor() 
{
    
}

void MotorControl::goForward(short motorSpeed)
{
    M1.setmotor(_CW, PWM);
    M2.setmotor(_CW, PWM);
    M3.setmotor(_CCW, PWM);
    M4.setmotor(_CCW, PWM);
}

void MotorControl::goBackward(short motorSpeed)
{
    M1.setmotor(_CCW, PWM);
    M2.setmotor(_CCW, PWM);
    M3.setmotor(_CW, PWM);
    M4.setmotor(_CW, PWM);
}

void MotorControl::goRight(short motorSpeed)
{
    M1.setmotor(_CCW, PWM);
    M2.setmotor(_CW, PWM);
    M3.setmotor(_CCW, PWM);
    M4.setmotor(_CW, PWM);
}

void MotorControl::goLeft(short motorSpeed)
{
    M1.setmotor(_CW, PWM);
    M2.setmotor(_CCW, PWM);
    M3.setmotor(_CW, PWM);
    M4.setmotor(_CCW, PWM);
}

void MotorControl::turnLeft(short motorSpeed)
{
    M1.setmotor(_CCW, PWMR);
    M2.setmotor(_CCW, PWMR);
    M3.setmotor(_CCW, PWMR);
    M4.setmotor(_CCW, PWMR);
}

void MotorControl::turnRight(short motorSpeed)
{
    M1.setmotor(_CW, PWMR);
    M2.setmotor(_CW, PWMR);
    M3.setmotor(_CW, PWMR);
    M4.setmotor(_CW, PWMR);
}

void MotorControl::stopMovement()
{
    M1.setmotor(_CW, 0);
    M2.setmotor(_CW, 0);
    M3.setmotor(_CCW, 0);
    M4.setmotor(_CCW, 0);
}


