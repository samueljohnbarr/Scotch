#include "main.h"
#include "lidar.h"
#include "wireless.h"
#include "chassis.h"
#include <math.h>

int currAngle = 0;

void calcTarget(int target[2]);

void navigate() {
  double velocities[2];
  int target[2];
  //while (1) {
  //Send Current Data
  int * dist = lidarGetDistances();
  print("***************Sending Lidar Data****************\n");
  wirelessSend(dist, 360);
  print("**************Sending Target Data****************\n");
  calcTarget(target);
  wirelessSend(target, 2);
  print("**************Recieving Velocities***************\n");
  wirelessRecieve(velocities);

  //Set Velocities
  print("**************Setting motor speeds***************\n");
  printf("Inside navigate: %f, %f\n", velocities[0], velocities[1]);
  velocitySet(velocities[0], velocities[1]);

  //Delay for lidar distance update
  delay(500);
  //}
}


void calcTarget(int target[2]) {
  int left = encoderGet(leftEncoder);
  int right = encoderGet(rightEncoder);

  //Convert to cm
  double leftCm = left / 9.5; //10.8
  double rightCm = right / 9.5;

  //Calculate angle by Pythagorean using the difference
  double diff = leftCm - rightCm;
  double diffAngle = atan2(diff, WIDTH_OF_BOT) * (180 / 3.14159);

  currAngle += diffAngle;

  //Correct currAngle if needed
  if (currAngle > 360) {
    currAngle %= 360;
  }
  if (currAngle > 180) {
     currAngle = -180 + (currAngle - 180);
  }
  else if (currAngle < -180) {
    currAngle = 180 + (currAngle + 180);
  }

  target[0] = currAngle;
  target[1] = 300; //Stub

  //Reset encoder values
  encoderReset(leftEncoder);
  encoderReset(rightEncoder);

}
/*
void calcTarget(int target[2])
{
  //Grab encoder values
  int left = encoderGet(leftEncoder);
  int right = encoderGet(rightEncoder);

  //Calc angle
  int diff = left - right;

  //Alter target
  target[0] = prevAngle + diff;
  target[1] = 300;


  //Reset encoders
  encoderReset(leftEncoder);
  encoderReset(rightEncoder);
}
*/
