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
    motorSet(RR_DRIVE, -speed);
    motorSet(FR_DRIVE, -speed);
  }
}

void velocitySet(double translational, double rotational) {
  //Change sign of rotational
  rotational = -rotational;
  if (translational < 0) translational = 0;//-translational;
  //Scale Values
  rotational = rotational * MAX_SPEED * 4.5;
  translational = translational * MAX_SPEED * 4.5;
  printf("Rotational: %f\n", rotational);
  printf("Translational: %f\n", translational);
  printf("Left motor set: %d\n", (int)(translational+rotational));
  printf("Right motor set: %d\n", (int)(translational-rotational));

  //Set speeds
  leftMotorSet((int)(translational + rotational));
  rightMotorSet((int)(translational - rotational));
}
