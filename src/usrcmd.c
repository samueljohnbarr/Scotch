/**
 * File used to decode and service user commands from terminal
 */
 #include "main.h"
 #include "chassis.h"
 #include "lidar.h"

/**
 * Naive implementation, but it works for these purposes
 * (if two strings are the same, but one has less characters
 *  than the other, it will still return true)
 */
int strIsEqual(const char * str1, const char * str2) {
   for (int i = 0; str1[i] != '\0' && str2[i] != '\0'; i++) {
     if (str1[i] != str2[i])
       return FALSE;
   }
   return TRUE;
}

void checkCmds() {
  if (fcount(stdin) > 1) {
    char in[20];
    fgets(in, 15, stdin);

    if (strIsEqual("lidarScan", in)) {
       int state = taskGetState(lidar);
       //If the lidar thread is not running, manual scan
       if (state == TASK_DEAD || state == TASK_SUSPENDED) {
         lidarFlushBuff();
         lidarScan();
         print("Scanned.\n");
       }
       else {
         print("Lidar is currently scanning on separate thread.\n");
         print("To kill that thread, type 'threadKill'\n");
       }
       return;
    }

    if (strIsEqual("threadKill", in)) {
      int state = taskGetState(lidar);
      //If the lidar thread is not running, ignore
      if (state == TASK_DEAD)
        print("Lidar thread already dead.\n");
      else {
        taskDelete(lidar);
        lidarStopScan();
        lidarFlushBuff();
        print("Lidar task stabbed to death.\n");
        print("Type 'threadStart' to revive the thread.\n");
      }
      return;
    }

    if (strIsEqual("threadStart", in)) {
      int state = taskGetState(lidar);
      //If the lidar thread is not running and not dead, resume
      if (state == TASK_DEAD || state == TASK_SUSPENDED) {
        lidar = taskCreate(lidarThreadedScan, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
        print("Lidar thread created and running.\n");
      }
      else
        print("Thread is already alive and well.\n");
      return;
    }

    if (strIsEqual("lidarPrint", in)) {
      print("Printing Distances...\n");
      lidarPrintDistances();
      return;
    }

    if (strIsEqual("lidarHealth", in)) {
      print("Getting Health...\n");
      int state = taskGetState(lidar);
      //If the lidar thread is not running and not dead, resume
      if (state == TASK_DEAD || state == TASK_SUSPENDED) {
        lidarFlushBuff();
        lidarGetHealth();
      }
      else {
        print("Lidar thread running.  Kill thread and try again.\n");
        print("Suspend thread with 'threadKill'\n");
      }
      return;
    }

    if (strIsEqual("lidarShutdown", in)) {
      taskDelete(lidar);
      lidarShutdown();
      lidarFlushBuff();
      print("Lidar shutdown.\n");
    }

    if (strIsEqual("lidarStartUp", in)) {
      lidarFlushBuff();
      initLidar();
      lidar = taskCreate(lidarThreadedScan, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
      print("Lidar booted.\n");
    }

    if (strIsEqual("send", in)) {
      print("Connectionless sending distance data");
      wirelessSend();
    }

  }
}
