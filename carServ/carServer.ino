#include <SPI.h>
#include <WiFi101.h>

#define TEST false // make true to run tests and run infinite loop

#define S_SSID "Kal" // name of network trying to connect to
#define SPASS "R3slif3sux42069"
#define B_SSID "Brown-Guest"
char bssid[] = B_SSID;
char ssid[] = S_SSID;        // your network SSID (name)
char pass[] = SPASS;
int status = WL_IDLE_STATUS;     // the WiFi server's status
WiFiServer server(8888); // Our server on port 8888

int RESET_PIN = 2;
int FORWARD_PIN = 4; // button pin to move forward
int BACKWARD_PIN = 7; // button pin to move backward
int LEFT_PIN = 5;
int RIGHT_PIN = 6;

enum SERV_STATE {
  WAIT,
  SEND
};

SERV_STATE currState = WAIT;


/*********** BUTTON AND INTERRUPTS ************/

#define R 0x1; 
#define R_BIT 0;
#define L 0x2; 
#define L_BIT 1;
#define B 0x4; 
#define B_BIT 2;
#define F 0x8; 
#define F_BIT 3;

volatile uint8_t buttons = 0;

volatile bool canInterruptF;
volatile unsigned long lastInterruptF = 0;
void updateButtonF() {

  // TODO: just keep track of last one you pressed duh
 // int t = millis();
  if (canInterruptF) {
    lastInterruptF = millis();
    canInterruptF = false;
    buttons ^= 1 << F_BIT;
  }
  
}

volatile bool canInterruptB;
volatile unsigned long lastInterruptB = 0;
void updateButtonB() {
  if (canInterruptB) {
    lastInterruptB = millis();
    canInterruptB = false;
    buttons ^= 1 << B_BIT;
  }
}

volatile bool canInterruptL;
volatile unsigned long lastInterruptL = 0;
void updateButtonL() {
  if (canInterruptL) {
    lastInterruptL = millis();
    canInterruptL = false;
    buttons ^= 1 << L_BIT;
  }
}

volatile bool canInterruptR;
volatile unsigned long lastInterruptR = 0;
void updateButtonR() {
  if (canInterruptR) {
    lastInterruptR = millis();
    canInterruptR = false;
    buttons ^= 1 << R_BIT;
  }
}


/*********** MAIN CODE ************/

void setup() {
  //Initialize serial and wait for port to open:
  initPins();
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }

   if (TEST) {
    runTests();
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to open SSID: ");
    Serial.println(bssid);
    //status = WiFi.begin(ssid, pass);
    status = WiFi.begin(bssid);

    // wait 10 seconds for connection:
    delay(10000);
  }


  
  // begin accepting clients and connect to the server
  startServer();
}


void updateFsm(int b) {
  if (currState == WAIT) {
    if (b != 0) {
      currState = SEND;
      return;
    }

    return;
  }

  if (currState == SEND) {
    if (b == 0) {
      currState = WAIT;
      return;
    }

    WiFiClient client = server.available(); // should this be here??
    if (client) {
      Serial.println("woooo client");
    }

    delay(90);
    Serial.println(b, BIN);
    server.write(b);
    return;
  }
}



void checkReset() {
  if (digitalRead(RESET_PIN)) {
    Serial.println("Resetting buttons...");
    buttons = 0;
  }
}

void loop() {
  delay(20);
  checkReset();

  int t = millis();
  if ((t - lastInterruptF) > 15) {
    canInterruptF = true;
  }

  if ((t - lastInterruptB) > 15) {
    canInterruptB = true;
  }

  if ((t - lastInterruptR) > 15) {
    canInterruptR = true;
  }

  if ((t - lastInterruptL) > 15) {
    canInterruptL = true;
  }

  updateFsm(buttons);
}

void initPins() {
  pinMode(FORWARD_PIN, INPUT);
  pinMode(BACKWARD_PIN, INPUT);
  pinMode(LEFT_PIN, INPUT);
  pinMode(RIGHT_PIN, INPUT);
  pinMode(RESET_PIN, INPUT);

    // change the buttons we are sending when button is pressed
  attachInterrupt(digitalPinToInterrupt(FORWARD_PIN), updateButtonF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BACKWARD_PIN), updateButtonB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_PIN), updateButtonR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_PIN), updateButtonL, CHANGE);
}

void startServer() {
  Serial.println("Connected to the network");
  server.begin();
  Serial.print("Starting server, IP: ");
  IPAddress myAddr = WiFi.localIP();
  Serial.println(myAddr);
  Serial.println("Now accepting clients");
}


/**************** TESTING ************************/


void testFSM(int buttons, SERV_STATE startState, SERV_STATE endState) {

  currState = startState;
  updateFsm(buttons);
  Serial.print("Tested with button input of: ");
  Serial.print(buttons, BIN);
  const char* c = (endState == currState) ? "SUCCEEDED" : "FAILED";
  Serial.print("... Test ");
  Serial.println(c);
}

void runTests() {

  // buttons = 0
  testFSM(0, WAIT, WAIT);
  testFSM(0, SEND, WAIT);

  // buttons != 0
  testFSM(1, SEND, SEND);
  testFSM(4, SEND, SEND);
  testFSM(1, WAIT, SEND);
  testFSM(4, WAIT, SEND);
  while (1) { };
}
