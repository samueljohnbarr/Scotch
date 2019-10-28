/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"
#include <stdlib.h>
#include "chassis.h"
#include "lidar.h"
#include "wireless.h"
#include "usrcmd.h"

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl() {
	int power;
	int turn;
	while (1) {
		power = joystickGetAnalog(CONTROLLER, R_JOY_V);
    if (power > -25 && power < 25)
      power = 0;
    turn = joystickGetAnalog(CONTROLLER, R_JOY_H);
    if (turn > -25 && turn < 25)
      turn = 0;

    //int heading = getHeading(power, turn);

    //if (isInRange(heading)) {
    //  printf("In range!\n\n");
    //  blockedHeading = heading;
    //  chassisSet(0,0);
    chassisSet(power + turn, power - turn);


		//**** RIGHT BUTTON SET *****
		if (joystickGetDigital(CONTROLLER, RIGHT_BUTT_SET, UP_BUTT)) {
      initLidar();
		}

		if (joystickGetDigital(CONTROLLER, RIGHT_BUTT_SET, DOWN_BUTT)) {
			lidarScan();
		}

		if (joystickGetDigital(CONTROLLER, RIGHT_BUTT_SET, LEFT_BUTT)) {
      lidarPrintDistances();
			delay(100); //Debounce
		}

		if (joystickGetDigital(CONTROLLER, RIGHT_BUTT_SET, RIGHT_BUTT)) {
      lidarShutdown();
		  delay(100);
		}


		//**** LEFT BUTTON SET ****
		if (joystickGetDigital(CONTROLLER, LEFT_BUTT_SET, UP_BUTT)) {
			//Prepare lidar data
			lidarScan();
			int * dist = lidarGetDistances();
			for (int i = 0; i < 360; i++) {
				printf("%d\n", dist[i]);
			}
			//Send lidar data
			printf("Sending lidar data");
			wirelessSend(dist, 360);

			//Send target data
			printf("Sending target data");
			int target[2] = {0, 300};
			wirelessSend(target, 2);
		}


		if (joystickGetDigital(CONTROLLER, LEFT_BUTT_SET, DOWN_BUTT)) {
			usartInit(uart2, 9600, SERIAL_8N1);
      double velocities[2];
			wirelessRecieve(velocities);
			printf("Velocities: %lf | %lf\n", velocities[0], velocities[1]);


		}


		if (joystickGetDigital(CONTROLLER, LEFT_BUTT_SET, LEFT_BUTT)) {

		}
		if (joystickGetDigital(CONTROLLER, LEFT_BUTT_SET, RIGHT_BUTT)) {

		}

    checkCmds();
		delay(20);
  }
}
