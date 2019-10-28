#include "lidar.h"
#include "main.h"

#define BAUD_RATE 115200
#define LIDAR_MOTOR_PIN 12
#define RED_LED_PIN 3
#define DEGREES 360

#define REQUEST_LENGTH 9 //Length of request (other than 2)
#define HEADER_LENGTH 7  //Length of response header
#define SCAN_PACKET_LENGTH 5 //Size of a single measurement (packet) in bytes

/** Verbose Mode - Set to 1 for verbose output **/
#define VERBOSE 0

/** Timeout - defines amount of time in ms before timing out during communication **/
#define TIMEOUT 1000

/** Resets - Amount of auto-resets before giving up on life **/
int resets = 10;

int t = 0; //For threading - denotes if scan should check header or not - Don't change

/**
 * Retrieves raw data from lidar one packet at a time.
 * Sends data to decode packet
 *
 * @param t set to 0 if you expect a response header, 1 otherwise.
 * For theading, header checking need only occur once.
 */
int getScanResponse(int t);

/**
 * Decodes a single packet and inserts it into the global array
 */
void decodePacket(int quality, int angle, int angle1, int dist, int dist1);

/**
 * Led control wrappers - used to signify an error has occured
 */
void ledOn(); void ledOff();

/**
 * Retrieves a byte from uart1 with a timeout function.
 *
 * @param b will be set to the retrieved byte
 * @return 0 on success, -1 on timeout
 */
int getByte(char * b);

/**
 * In the event of communication failure or bad health, recovery will attempt
 * to reset the lidar until the issue is resolved or if it runs out of resets
 */
int recover();

char scanHeader[]   = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};
char infoHeader[]   = {0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04};
char healthHeader[] = {0xA5, 0x5A, 0x3, 0x00, 0x00, 0x00, 0x06};
char rateHeader[]   = {0xA5, 0x5A, 0x4, 0x00, 0x00, 0x00, 0x15};
int lidarDistances[DEGREES];


int initLidar() {
  //pinmode setup found in init.c
  usartInit(uart1, BAUD_RATE, SERIAL_8N1);
  digitalWrite(LIDAR_MOTOR_PIN,HIGH);
  int health = lidarGetHealth();
  if (health > 0)
    recover();
  return health;
}

void lidarScan() {
  long int before = millis();
  //Send request
  fputc(0xA5, uart1);
  fputc(0x20, uart1);
  getScanResponse(0);
  long int after = millis();
  if (VERBOSE) printf("Scan Retrieval Time: %ld ms", after-before);
  lidarStopScan();
}

void lidarThreadedScan(void * n) {
  delay(1000);
  //Send request
  fputc(0xA5, uart1);
  fputc(0x20, uart1);

  //Loop Response
  while (true) {
    getScanResponse(t);
    if (t == 0)
      t++;
  }
  lidarStopScan();
}

int getScanResponse(int t) {
  //Verify response header
  int i;
  char curr;
  if (!t)
    for (i = 0; i < HEADER_LENGTH; i++) {
      getByte(&curr);
      if (curr != scanHeader[i]) {
        print("getScanResponse: Header Mismatch Exception\n");
        return -1;
      }
    }

  int numBytes = DEGREES * SCAN_PACKET_LENGTH;
  int quality, angle, angle1, dist, dist1 = 0;

  //Grab packets from lidar
  for (i = 0; i < numBytes; i++) {
    if (!(i % 5))       quality = fgetc(uart1);
    if ((i % 5) == 1)   angle = fgetc(uart1);
    if ((i % 5) == 2)   angle1 = fgetc(uart1);
    if ((i % 5) == 3)   dist = fgetc(uart1);
    if ((i % 5) == 4) { dist1 = fgetc(uart1);
      //Once a single packet retreived, decode it
      if (VERBOSE) {
        printf("%x | %x | %x | %x | %x\n", quality, angle, angle1, dist, dist1);
        delay(20);
      }
      decodePacket(quality, angle, angle1, dist, dist1);
    }
  }
  return true;
}

void decodePacket(int quality, int angle, int angle1, int dist, int dist1) {
  int currQuality = quality >> 2;
  //Shift angle2 over to make room for angle1
  int currAngle = angle1 << 8;
  //Bitwise OR angle2 with angle 1, clearing off upper bits of angle1 first
  currAngle = currAngle | (0xFF & angle);
  //Shift resultant angle over 1 to clear of C bit
  currAngle = (currAngle >> 1) & 0x7FFF;
  //Shift distance2 over to make room for distance1
  int currDist = dist1 << 8;
  //Bitwise OR to get the distance together
  currDist = (currDist | ( 0xFF & dist));
  //Clear off upper bits
  currDist &= 0xFFFF;

  //Calculate actual angle and distance
  currAngle = currAngle/64;
  currDist = currDist/40;

  if (currAngle < DEGREES && currAngle >= 0) {
    //Only keep quality data
    //if (currQuality == 15) {
      //if (currDist < F_RANGE && currDist && (currAngle <= 90 || currAngle >= 270)) setQuadrant(currAngle, currDist); //&&
    //  if (currDist < R_RANGE && currDist && (currAngle > 90 && currAngle < 270)) setQuadrant(currAngle, currDist);
         //((currAngle >= 0 && currAngle <= 90) || (currAngle >= 270 && currAngle <= 360)))
         //localQuad[0] = 1;
      lidarDistances[currAngle] = currDist;
    //}
    //else
      //If quality is shite, set distance to zero
    //  lidarDistances[currAngle] = 0;
  }
  if (VERBOSE) printf("Angle: %d | Quality: %d | Distance: %d\n", currQuality, currAngle, currDist);
}

void lidarGetInfo() {
  //Send request
  fputc(0xA5, uart1);
  fputc(0x50, uart1);

  //Verify header
  char curr;
  for (int i = 0; i < HEADER_LENGTH; i++) {
    getByte(&curr);
    if (curr != infoHeader[i]) {
      printf("lidarGetInfo: Header Mismatch Exception\n");
      return;
    }
  }

  //Retrieve data
  char model      = fgetc(uart1);
  char firm_minor = fgetc(uart1);
  char firm_major = fgetc(uart1);
  char hardware   = fgetc(uart1);
  char serial1    = fgetc(uart1);
  char serial2    = fgetc(uart1);

  //Join serial numbers
  unsigned int serialno = serial2 << 8;
  serialno = (serialno | (0xFF & serial1));

  //Print
  printf("-----------Lidar Information----------\n");
  printf("Model ID: %d\n", model);
  printf("Firmware Version: %d.%d\n", firm_major, firm_minor);
  printf("Hardware Version: %d\n", hardware);
  printf("Serial Number: %u\n", serialno);
  printf("--------------------------------------\n\n");
}

int lidarGetHealth() {
  //Send request
  fputc(0xA5, uart1);
  fputc(0x52, uart1);

  //Verify header
  for (int i = 0; i < HEADER_LENGTH; i++) {
    char curr;
    getByte(&curr);
    if (curr != healthHeader[i]) {
      printf("lidarGetHealth: Header Mismatch Exception\n");
      printf("buffer: %x | header: %x\n", curr, healthHeader[i]);
      return -1;
    }
  }

  //Retrieve data
  char status;
  getByte(&status);
  char errcode1 = fgetc(uart1);
  char errcode2 = fgetc(uart1);

  //Join error codes
  unsigned int errorcode = errcode2 << 8;
  errorcode = (errorcode | (0xFF & errcode1));

  //Print
  printf("--------------Lidar Health-------------\n");
  printf("Status: ");
  if (!status)
    printf("Good - Lidar is in good health\n");
  else if (status == 1)
    printf("Warning - System may work as normal, but there exists a potential risk\n");
  else if (status == 2)
    printf("Error - Lidar is in Protection Stop state | Refer to error code for more info\n");

  if (status) printf("Error Code: %u\n", errorcode);
  printf("---------------------------------------\n\n");

  return status;
}

void lidarGetSampleRate() {
  //Send request
  fputc(0xA5, uart1);
  fputc(0x59, uart1);

  //Verify header
  char curr;
  for (int i = 0; i < HEADER_LENGTH; i++) {
    getByte(&curr);
    if (curr != rateHeader[i]) {
      printf("lidarGetSampleRate: Header Mismatch Exception\n");
      return;
    }
  }
  //Retrieve data
  char stand1 = fgetc(uart1);
  char stand2 = fgetc(uart1);
  char expr1 = fgetc(uart1);
  char expr2 = fgetc(uart1);

  //Join rates
  unsigned int standard = stand2 << 8;
  standard = (standard | (0xFF & stand1));
  unsigned int express = expr2 << 8;
  express = (express | (0xFF & expr1));

  //Print
  printf("-----------Lidar Samplerate---------\n");
  printf("Standard Scan: %d microseconds\n", standard);
  printf("Express Scan:  %d microseconds\n", express);
  printf("------------------------------------\n");
}

void lidarStopScan() {
  if (VERBOSE) printf("Lidar Stop Scan...\n");
  fputc(0xA5, uart1);
  fputc(0x25, uart1);
  delay(2);
}

void lidarStartScan() {
  fputc(0xA5, uart1);
  fputc(0x20, uart1);
}

void lidarReset() {
  if (VERBOSE) printf("Lidar Reset...\n");
  fputc(0xA5, uart1);
  fputc(0x40, uart1);
  resets--;
  delay(3);
}

void lidarShutdown() {
  if (VERBOSE) printf("Lidar Shutdown...\n");
  //Stop scanning
  lidarStopScan();
  //Shutdown usart communication
  usartShutdown(uart1);
  //Stop lidar motor
  digitalWrite(LIDAR_MOTOR_PIN, LOW);
}

int * lidarGetDistances() {
  return lidarDistances;
}

void lidarPrintDistances() {
  for (int i = 0; i < DEGREES; i++) {
    printf("%d\n", lidarDistances[i]);
    delay(20);
  }
}

void ledOn() {
  digitalWrite(RED_LED_PIN, LOW);
}

void ledOff() {
  digitalWrite(RED_LED_PIN, HIGH);
}


int getByte(char * b) {
    unsigned long int timeout = millis() + TIMEOUT;
    while (!fcount(uart1) && millis() < timeout);
    if (!fcount(uart1)) {
      if (VERBOSE) printf("getByte: Timeout Exception\n");
      if (recover()) {
        printf("Fatal: Communication Failure - Exiting...\n");
        lidarShutdown();
        exit(1);
      }
    }
    *b = fgetc(uart1);
    return 0;
}

void lidarFlushBuff() {
  while(fcount(uart1)) {
    fgetc(uart1);
  }
}

int recover() {
  ledOn();
  //Attempt to powercycle the lidar
  do {
    lidarShutdown();
    delay(100);
    initLidar();
    delay(100);
    resets--;
  } while (!lidarGetHealth() && resets >= 0);
  //If loop exited with resets left, we good
  if (resets) {
    ledOff();
    return 0;
  }
  return -1;
}
