#include <SPI.h>
#include <WiFi101.h>


/********** WIFI **********/

#define SECRET_SSID "Brown-Guest"

char ssid[] = SECRET_SSID;  // your network SSID (name)
int status = WL_IDLE_STATUS;  // the WiFi radio's status

WiFiClient client;
IPAddress ip(172, 18, 148, 85);  // brown guest
#define connPort 8888


/********** I/O **********/

int LEFT_MOTOR_PIN1 = 2;
int LEFT_MOTOR_PIN2 = 1;
int LEFT_MOTOR_PWM = 0;

int RIGHT_MOTOR_PIN1 = 3;
int RIGHT_MOTOR_PIN2 = 4;
int RIGHT_MOTOR_PWM = 5;

int COLOR_S0 = 6;
int COLOR_S1 = 7;
int COLOR_S2 = 8;
int COLOR_S3 = 9;
int COLOR_OUT = 10;


/********** DIRECTIONS **********/

#define R 0x1
#define R_BIT 0
#define L 0x2
#define L_BIT 1
#define B 0x4
#define B_BIT 2
#define F 0x8
#define F_BIT 3


/********** FSM ENUMS **********/

enum CAR_STATE {
  WAIT_FOR_INPUT,
  MOTOR_COMMAND,
  STOP_AFTER_RED
};

enum COLOR {
  NONE,
  RED,
  BLUE,
  GREEN
};


/********** CONSTANTS **********/

const int RED_THRESHOLD = 5000;
const int RED_STOP_TIME = 2000;
const int GREEN_BOOST_TIME = 1000;
const int BLUE_SLOW_TIME = 1000;

const int MOTOR_DEFAULT_SPEED = 170;
const int MOTOR_BOOST_SPEED = 250;
const int MOTOR_SLOW_SPEED = 100;


/********** MOTOR FUNCTION OPTIONS **********/

enum motorFunction {
  OFF,
  FORWARD,
  BACKWARD
};


/********** FSM VARIABLES **********/

CAR_STATE currState;
int LAST_RED;
int CURR_DIR;
COLOR CURR_COLOR;


/********** TESTING VARIABLES **********/

#define TEST false

struct fsmVariables {
  int lastRed;
  int currDir;
  COLOR currColor;
};

enum motorSpeed {
  SLOW,
  NORMAL,
  FAST
};

struct motorOutput {
  motorFunction leftMotor;
  motorFunction rightMotor;
  motorSpeed speed;
};

motorOutput currOutput = {OFF, OFF, NORMAL};



void initPins() {
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);

  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  pinMode(COLOR_S0, OUTPUT);
  pinMode(COLOR_S1, OUTPUT);
  pinMode(COLOR_S2, OUTPUT);
  pinMode(COLOR_S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);

  // Setting frequency scaling to 2%
  digitalWrite(COLOR_S0, LOW);
  digitalWrite(COLOR_S1, HIGH);
}



void initFSM() {
  currState = WAIT_FOR_INPUT;
  LAST_RED = -5000;
  CURR_DIR = 0x0;
  CURR_COLOR = NONE;
}



void setup() {
  Serial.begin(9600);
  if (TEST) while (!Serial);  // wait for serial port to connect. Needed for native USB port only
  initPins();
  initFSM();

  if (TEST) {
    Serial.println("TESTING");
    runTests();
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to open SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid);  // brown wifi

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the data:
  Serial.println("Connected to the network :D");

  while (!client.connect(ip, connPort)) {
    Serial.println("error connecting to serve :(");
  }

  Serial.println("Connected to server");

  // setup the Watchdog
  WDT_setup();
}



void loop() {
  // if client disconnects
  while (!client.connected()) {
    // reconnect to server
    WDT_pet();
    Serial.println("disconnected, attempting to reconnect");
    client.connect(ip, connPort);
    updateFSM(0, NONE, millis());
    delay(1000);
  }

  WDT_pet();
  COLOR color = pollColorSensor();
  char direction = 0;

  // if server wrote byte
  if (client.available()) {
    direction = client.read();
  }

  updateFSM(direction, color, millis());
  delay(12);
}



void updateFSM(int direction, COLOR color, int time) {
  if (currState == WAIT_FOR_INPUT) {
    if ((color == RED) && (time - LAST_RED > RED_THRESHOLD)) {  // transition 1-3
      setMotors(direction, color);
      LAST_RED = time;
      currState = STOP_AFTER_RED;
      return;
    }
    if ((color != RED)) {  // transition 1-2 (a)
      CURR_DIR = direction;
      CURR_COLOR = color;
      currState = MOTOR_COMMAND;
      return;
    }
    if ((color == RED) && (time - LAST_RED <= RED_THRESHOLD)) { // transition 1-2 (b)
      CURR_DIR = direction;
      CURR_COLOR = NONE;
      currState = MOTOR_COMMAND;
      return;
    }
    return;
  }

  if (currState == MOTOR_COMMAND) {  // transition 2-1
    setMotors(CURR_DIR, CURR_COLOR);
    currState = WAIT_FOR_INPUT;
    return;
  }

  if (currState == STOP_AFTER_RED) {
    if (time - LAST_RED >= RED_STOP_TIME) {  // transition 3-1
      LAST_RED = time;
      currState = WAIT_FOR_INPUT;
      return;
    }
    return;
  }
}



COLOR pollColorSensor() {
  // Setting RED (R) filtered photodiodes to be read
  digitalWrite(COLOR_S2, LOW);
  digitalWrite(COLOR_S3, LOW);

  // Reading the output frequency
  int redFrequency = pulseIn(COLOR_OUT, LOW);

  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(COLOR_S2, HIGH);
  digitalWrite(COLOR_S3, HIGH);

  // Reading the output frequency
  int greenFrequency = pulseIn(COLOR_OUT, LOW);

  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(COLOR_S2, LOW);
  digitalWrite(COLOR_S3, HIGH);

  // Reading the output frequency
  int blueFrequency = pulseIn(COLOR_OUT, LOW);

  if (redFrequency < blueFrequency && redFrequency <= greenFrequency && redFrequency < 1200) {
      return RED;
  }
  else if (blueFrequency < greenFrequency && blueFrequency < redFrequency && blueFrequency < 2000) {
      return BLUE;
  }
  else if (greenFrequency < blueFrequency && greenFrequency - redFrequency <= 400) {
      return GREEN;
  }
  else {
      return NONE;
  }
}



void setMotors(char direction, COLOR color) {
  int motorSpeed = MOTOR_DEFAULT_SPEED;
  if (TEST) currOutput.speed = NORMAL;

  // if no button is pressed or color is red or both forward and back are pressed then turn motors off
  if (direction == 0x0 || color == RED || ((direction & F) && (direction & B)) || !((direction & F) || (direction & B))) {
    powerLeftMotors(OFF);
    powerRightMotors(OFF);
    return;
  }

  // set motor speed based on color
  if (color == GREEN) {
    motorSpeed = MOTOR_BOOST_SPEED;
    if (TEST) currOutput.speed = FAST;
  }
  if (color == BLUE) {
    motorSpeed = MOTOR_SLOW_SPEED;
    if (TEST) currOutput.speed = SLOW;
  }

  analogWrite(LEFT_MOTOR_PWM, motorSpeed);
  analogWrite(RIGHT_MOTOR_PWM, motorSpeed);

  // set motors to move forward
  if (direction & F) {
    powerLeftMotors(FORWARD);
    powerRightMotors(FORWARD);
  }

  // set motors to move backward
  if (direction & B) {
    powerLeftMotors(BACKWARD);
    powerRightMotors(BACKWARD);
  }

  // if both left and right are pressed don't change direction
  if ((direction & R) && (direction & L)) return;

  // if right pressed then turn off right motors
  if (direction & R) {
    powerRightMotors(OFF);
  }

  // if left pressed then turn off left motors
  if (direction & L) {
    powerLeftMotors(OFF);
  }
}


void powerLeftMotors(motorFunction function) {
  if (TEST) {
    currOutput.leftMotor = function;
    return;
  }

  int motor1 = LOW;
  int motor2 = LOW;
  switch(function) {
    case FORWARD:
      motor1 = HIGH;
      break;
    case BACKWARD:
      motor2 = HIGH;
      break;
  }
  
  digitalWrite(LEFT_MOTOR_PIN1, motor1);
  digitalWrite(LEFT_MOTOR_PIN2, motor2);
}



void powerRightMotors(motorFunction function) {
  if (TEST) {
    currOutput.rightMotor = function;
    return;
  }

  int motor1 = LOW;
  int motor2 = LOW;
  switch(function) {
    case FORWARD:
      motor1 = HIGH;
      break;
    case BACKWARD:
      motor2 = HIGH;
      break;
  }

  digitalWrite(RIGHT_MOTOR_PIN1, motor1);
  digitalWrite(RIGHT_MOTOR_PIN2, motor2);
}

/********** WATCHDOG CODE ***************************/
void WDT_setup() {
  // Clear and enable WDT
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0);
  NVIC_EnableIRQ(WDT_IRQn);

  // Configure WDT GCLK
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(4) | GCLK_GENDIV_ID(5);
  while (GCLK->STATUS.bit.SYNCBUSY);

  // set GCLK->GENCTRL.reg and GCLK->CLKCTRLL.reg
  GCLK->GENCTRL.reg = GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_ID(0x05) | 
                      GCLK_GENCTRL_GENEN | GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.bit.SYNCBUSY);

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(0x03) | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK5;

  // configure and enable WDT
  WDT->CONFIG.reg = 0x8; // period of 2048 clock cycles, around 1 second with 1kHz clock
  WDT->EWCTRL.reg = 0x7; // early warning at 1 sec, period of 1024
  WDT->CTRL.reg = WDT_CTRL_ENABLE;
  while(WDT->STATUS.bit.SYNCBUSY);
  WDT->INTENSET.bit.EW = 1;
  
}


void WDT_pet() {
  WDT->CLEAR.reg = 0xa5;
}

void WDT_Handler() {
  WDT->INTFLAG.reg = 1;
  Serial.println("Watchdog about to be reset...");
}


/********** ALL TESTING RELATED CODE BELOW **********/

bool testFSM(int testNum, CAR_STATE startState, CAR_STATE endState, int dir, COLOR color, int time, fsmVariables startVars, fsmVariables endVars, motorOutput output) {
  Serial.print("TEST CASE ");
  Serial.println(testNum);

  // INPUTS
  Serial.println("Testing with input of: ");

  Serial.print("\tdirection: \t");
  Serial.println(dir, BIN);

  Serial.print("\tcolor: \t\t");
  Serial.println(color);

  Serial.print("\ttime: \t\t");
  Serial.println(time);

  // UPDATING
  currState = startState;
  LAST_RED = startVars.lastRed;
  CURR_DIR = startVars.currDir;
  CURR_COLOR = startVars.currColor;

  updateFSM(dir, color, time);

  bool succeeded = true;

  // VARIABLES
  Serial.println("Variable results: ");

  Serial.print("\tlastRed: \t");
  Serial.print(startVars.lastRed);
  Serial.print("\t -> \t");
  Serial.print(LAST_RED);
  if (LAST_RED != endVars.lastRed) {
    Serial.print(" [INCORRECT expected ");
    Serial.print(endVars.lastRed);
    Serial.print("]");
    succeeded = false;
  }
  Serial.println("");

  Serial.print("\tcurrDir: \t");
  Serial.print(startVars.currDir, BIN);
  Serial.print("\t -> \t");
  Serial.print(CURR_DIR, BIN);
  if (CURR_DIR != endVars.currDir) {
    Serial.print(" [INCORRECT expected ");
    Serial.print(endVars.currDir, BIN);
    Serial.print("]");
    succeeded = false;
  }
  Serial.println("");

  Serial.print("\tcurrColor: \t");
  Serial.print(getColorEnumName(startVars.currColor));
  Serial.print("\t -> \t");
  Serial.print(getColorEnumName(CURR_COLOR));
  if (CURR_COLOR != endVars.currColor) {
    Serial.print(" [INCORRECT expected ");
    Serial.print(getColorEnumName(endVars.currColor));
    Serial.print("]");
    succeeded = false;
  }
  Serial.println("");

  // OUTPUTS
  Serial.print("\tleft motor: \t");
  Serial.print(getMotorEnumName(currOutput.leftMotor));
  if (currOutput.leftMotor != output.leftMotor) {
    Serial.print(" [INCORRECT expected ");
    Serial.print(getMotorEnumName(output.leftMotor));
    Serial.print("]");
    succeeded = false;
  }
  Serial.println("");

  Serial.print("\tright motor: \t");
  Serial.print(getMotorEnumName(currOutput.rightMotor));
  if (currOutput.rightMotor != output.rightMotor) {
    Serial.print(" [INCORRECT expected ");
    Serial.print(getMotorEnumName(output.rightMotor));
    Serial.print("]");
    succeeded = false;
  }
  Serial.println("");

  Serial.print("\tspeed: \t\t");
  Serial.print(getSpeedEnumName(currOutput.speed));
  if (currOutput.speed != output.speed) {
    Serial.print(" [INCORRECT expected ");
    Serial.print(getSpeedEnumName(output.speed));
    Serial.print("]");
    succeeded = false;
  }
  Serial.println("");

  // STATE
  Serial.println("State change: ");
  Serial.print(getStateEnumName(startState));
  Serial.print("\t -> \t");
  Serial.print(getStateEnumName(currState));
  if (currState != endState) {
    Serial.print(" [INCORRECT expected ");
    Serial.print(getStateEnumName(endState));
    Serial.print("]");
    succeeded = false;
  }
  Serial.println("");

  // SUCCESS
  Serial.print("Test ");
  Serial.print(testNum);
  Serial.print(" ");
  Serial.println(succeeded ? "SUCCEEDED" : "FAILED");
  Serial.println("");

  return succeeded;
}



String getStateEnumName(CAR_STATE state) {
  switch (state) {
    case WAIT_FOR_INPUT:
      return "WAIT_FOR_INPUT";
    case MOTOR_COMMAND:
      return "MOTOR_COMMAND";
    case STOP_AFTER_RED:
      return "STOP_AFTER_RED";
    default:
      return "INVALID STATE";
  }
}



String getSpeedEnumName(motorSpeed speed) {
  switch (speed) {
    case SLOW:
      return "SLOW";
    case NORMAL:
      return "NORMAL";
    case FAST:
      return "FAST";
    default:
      return "INVALID SPEED";
  }
}



String getColorEnumName(COLOR color) {
  switch (color) {
    case NONE:
      return "NONE";
    case GREEN:
      return "GREEN";
    case BLUE:
      return "BLUE";
    case RED:
      return "RED";
    default:
      return "INVALID COLOR";
  }
}



String getMotorEnumName(motorFunction function) {
  switch (function) {
    case OFF:
      return "OFF";
    case FORWARD:
      return "FORWARD";
    case BACKWARD:
      return "BACKWARD";
    default:
      return "INVALID FUNCTION";
  }
}



void runTests() {
  Serial.println("Running tests...\n");

  int numTests = 43;
  int numSuccess = 0;

  CAR_STATE startState[numTests] = {WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, STOP_AFTER_RED, STOP_AFTER_RED, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND};
  CAR_STATE endState[numTests] = {MOTOR_COMMAND, MOTOR_COMMAND, MOTOR_COMMAND, STOP_AFTER_RED, STOP_AFTER_RED, STOP_AFTER_RED, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT, WAIT_FOR_INPUT};
  int dir[numTests] = {0b0, 0b1010, 0b1010, 0b1010, 0b1010, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0};
  COLOR color[numTests] = {NONE, NONE, RED, RED, RED, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE};
  int time[numTests] = {0, 0, 6000, 1500, 8000, 8500, 10000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  fsmVariables startVars[numTests] = {{ 0  , 0b0 , NONE }, { 0  , 0b0 , NONE }, { 1500  , 0b0 , NONE }, { -5000  , 0b0 , NONE }, { 1500  , 0b0 , NONE }, { 8000  , 0b0 , NONE }, { 8000  , 0b0 , NONE }, { 0  , 0b1010 , NONE }, { 0  , 0b1010 , GREEN }, { 0  , 0b1010 , BLUE }, { 0  , 0b1001 , NONE }, { 0  , 0b1001 , GREEN }, { 0  , 0b1001 , BLUE }, { 0  , 0b110 , NONE }, { 0  , 0b110 , GREEN }, { 0  , 0b110 , BLUE }, { 0  , 0b101 , NONE }, { 0  , 0b101 , GREEN }, { 0  , 0b101 , BLUE }, { 0  , 0b1000 , NONE }, { 0  , 0b1000 , GREEN }, { 0  , 0b1000 , BLUE }, { 0  , 0b100 , NONE }, { 0  , 0b100 , GREEN }, { 0  , 0b100 , BLUE }, { 0  , 0b10 , NONE }, { 0  , 0b10 , GREEN }, { 0  , 0b10 , BLUE }, { 0  , 0b1 , NONE }, { 0  , 0b1 , GREEN }, { 0  , 0b1 , BLUE }, { 0  , 0b1100 , NONE }, { 0  , 0b1100 , GREEN }, { 0  , 0b1100 , BLUE }, { 0  , 0b11 , NONE }, { 0  , 0b11 , GREEN }, { 0  , 0b11 , BLUE }, { 0  , 0b1111 , NONE }, { 0  , 0b1111 , GREEN }, { 0  , 0b1111 , BLUE }, { 0  , 0b0 , NONE }, { 0  , 0b0 , GREEN }, { 0  , 0b0 , BLUE }};
  fsmVariables endVars[numTests] = {{ 0 , 0b0 , NONE }, { 0 , 0b1010 , NONE }, { 1500 , 0b1010 , NONE }, { 1500 , 0b0 , NONE }, { 8000 , 0b0 , NONE }, { 8000 , 0b0 , NONE }, { 10000 , 0b0 , NONE }, { 0 , 0b1010 , NONE }, { 0 , 0b1010 , GREEN }, { 0 , 0b1010 , BLUE }, { 0 , 0b1001 , NONE }, { 0 , 0b1001 , GREEN }, { 0 , 0b1001 , BLUE }, { 0 , 0b110 , NONE }, { 0 , 0b110 , GREEN }, { 0 , 0b110 , BLUE }, { 0 , 0b101 , NONE }, { 0 , 0b101 , GREEN }, { 0 , 0b101 , BLUE }, { 0 , 0b1000 , NONE }, { 0 , 0b1000 , GREEN }, { 0 , 0b1000 , BLUE }, { 0 , 0b100 , NONE }, { 0 , 0b100 , GREEN }, { 0 , 0b100 , BLUE }, { 0 , 0b10 , NONE }, { 0 , 0b10 , GREEN }, { 0 , 0b10 , BLUE }, { 0 , 0b1 , NONE }, { 0 , 0b1 , GREEN }, { 0 , 0b1 , BLUE }, { 0 , 0b1100 , NONE }, { 0 , 0b1100 , GREEN }, { 0 , 0b1100 , BLUE }, { 0 , 0b11 , NONE }, { 0 , 0b11 , GREEN }, { 0 , 0b11 , BLUE }, { 0 , 0b1111 , NONE }, { 0 , 0b1111 , GREEN }, { 0 , 0b1111 , BLUE }, { 0 , 0b0 , NONE }, { 0 , 0b0 , GREEN }, { 0 , 0b0 , BLUE }};
  motorOutput output[numTests] = {{ OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , FORWARD , NORMAL }, { OFF , FORWARD , FAST }, { OFF , FORWARD , SLOW }, { FORWARD , OFF , NORMAL }, { FORWARD , OFF , FAST }, { FORWARD , OFF , SLOW }, { OFF , BACKWARD , NORMAL }, { OFF , BACKWARD , FAST }, { OFF , BACKWARD , SLOW }, { BACKWARD , OFF , NORMAL }, { BACKWARD , OFF , FAST }, { BACKWARD , OFF , SLOW }, { FORWARD , FORWARD , NORMAL }, { FORWARD , FORWARD , FAST }, { FORWARD , FORWARD , SLOW }, { BACKWARD , BACKWARD , NORMAL }, { BACKWARD , BACKWARD , FAST }, { BACKWARD , BACKWARD , SLOW }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }, { OFF , OFF , NORMAL }};

  for (int i = 0; i < numTests; i++) {
    bool success = testFSM(i + 1, startState[i], endState[i], dir[i], color[i], time[i], startVars[i], endVars[i], output[i]);
    if (!success) {
      while(true);
    }
    numSuccess += success;
  }

  Serial.print("Passed ");
  Serial.print(numSuccess);
  Serial.print("/");
  Serial.print(numTests);
  Serial.println(" tests!");

  while (true);
}
