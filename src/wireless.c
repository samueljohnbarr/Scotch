#include "main.h"
#include "lidar.h"

char REQUEST = 0xA5;
char ACK = 0x5A;
char STOP = 0xB5;
char END = 0x5B;
char getSentChar();

void wirelessSend(int * payload, int size) {
    fputc(REQUEST, uart2);
    char ans = getSentChar();
    printf("Waiting for ACK...");
    if (ans == ACK) {
      printf("Ack recieved - sending payload");
      int buffer_size = 10;
      char buf[buffer_size];
      //Send size of payload to reciever
      snprintf(buf, buffer_size, "%d", size);
      fputs(buf, uart2);

      for (int i = 0; i < size; i++) {
        //Send that ish
        snprintf(buf, buffer_size, "%d\n", payload[i]);
        printf("Sending: %s", buf);
        fputs(buf, uart2);
      }
      printf("payload sent\n");
      fputc(STOP, uart2);
      ans = fgetc(uart2);
      if (ans != ACK) {
        printf("Reciever did not acknoledge transmission\n");
      }
      printf("Complete.\n");
    }
    else {
      printf("Reciever Denied/Ignored Request - Code: %x\n", ans);
    }
}


void wirelessRecieve(double ret[2]) {
  printf("Port waiting...\n");
  while (fcount(uart2) == 0) {
    //Do nothing
  }
  char ans[2][20];
  printf("Data found on port!\n");
  char req = getSentChar();
  printf("Request bit: %x\n", req);
  if (req == REQUEST) {
    //Acknowledge Request
    printf("Sending ACK...\n");
    fputc(ACK, uart2);
    printf("ACK sent - recieving payload...\n");
    char a;
    int i = 0;
    do
    {
      for (int j = 0; j < 20; j++)
      {
        a = fgetc(uart2);
        if (a == END || a == STOP) break;
        ans[i][j] = a;
      }
      i++;
    } while (a != STOP);
    //ACK stop bit
    fputc(ACK, uart2);
  }
  //Decode payload into double values
  char * ptr;
  ret[0] = strtod(ans[0], &ptr);
  ret[1] = strtod(ans[1], &ptr);
  printf("Payload retrieved\n");
}


/**
* Waits for an available character to be read from the
* specified serial port. Returns the character after it is read.
*/
char getSentChar() {
  unsigned long int timeout = 5000 + millis();
  //Wait for something on the buffer
  while(!fcount(uart2))
    //Return if this action takes more than 5 seconds
    if (millis() > timeout) return 0x00;
  return fgetc(uart2);
}
