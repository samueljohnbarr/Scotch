#include "main.h"
#include "lidar.h"
#include "wireless.h"

void navigate() {
  while (1) {
    //Send Current Data
    int * dist = lidarGetDistances();
    print("Sending Lidar Data\n");
    wirelessSend(dist, 360);

  }
}
