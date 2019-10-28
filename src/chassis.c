#include "main.h"
#define FL_DRIVE 9
#define RL_DRIVE 10
#define FR_DRIVE 2
#define RR_DRIVE 1

#define MAX_SPEED 127

void chassisSet(int leftSpeed, int rightSpeed) {
  motorSet(RL_DRIVE, -leftSpeed);
  motorSet(FL_DRIVE, leftSpeed);
  motorSet(RR_DRIVE, -rightSpeed);
  motorSet(FR_DRIVE, -rightSpeed);
}


void leftMotorSet(int speed) {
  if (speed > -128 && speed < 128) {
    motorSet(RL_DRIVE, -speed);
    motorSet(FL_DRIVE, speed);
  }
}

void rightMotorSet(int speed) {
  if (speed > -128 && speed < 128) {
    motorSet(RL_DRIVE, -speed);
    motorSet(FL_DRIVE, -speed);
  }
}

void velocitySet(float translational, float rotational) {
  //Change sign of rotational
  rotational = -rotational;

  //Scale Values
  rotational = rotational * MAX_SPEED/2;
  translational = translational * MAX_SPEED/2;

  //Set speeds
  leftMotorSet(translational + rotational);
  rightMotorSet(translational - rotational);
}
