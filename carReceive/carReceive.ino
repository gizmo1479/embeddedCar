#include <SPI.h>
#include <WiFi101.h>
// #include <avr/wdt.h>


/********** WIFI **********/

#define SECRET_SSID "Brown-Guest"
// #define SECRET_SSID "Kal"
// #define SECRET_PASS "R3slif3sux42069"

char ssid[] = SECRET_SSID;        // your network SSID (name)
// char pass[] = SECRET_PASS;        // your network password (name)
int status = WL_IDLE_STATUS;     // the WiFi radio's status

WiFiClient client;
IPAddress ip(172, 18, 142, 238); // brown guest
// IPAddress ip(192, 168, 0, 28); // home wifi
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
  STOP,
  WAIT_AFTER_STOP  
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


/********** FSM VARIABLES **********/

CAR_STATE currState;
int LAST_RED;
int CURR_DIR;
COLOR CURR_COLOR;


void setup() {
  Serial.begin(9600);
  // while (!Serial); // wait for serial port to connect. Needed for native USB port only

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to open SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid); // brown wifi
  //  status = WiFi.begin(ssid, pass); // home wifi

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the data:
  Serial.println("Connected to the network :D");

  while (!client.connect(ip, connPort)) {
    Serial.println("error connecting to serve :(");
  }

  Serial.println("Connected to server");

  initPins();
  initFSM();
  // wdt_enable(WDTO_250MS);
}

void loop() {
  // if client disconnects
  while (!client.connected()) {
    // reconnect to server
    Serial.println("disconnected, attempting to reconnect");
    client.connect(ip, connPort);
    updateFSM(0, NONE, millis());
    delay(1000);
  }

  COLOR color = pollColorSensor();
  char direction = 0;

  // if server wrote byte
  if (client.available()) {
    direction = client.read();
  }

  updateFSM(direction, color, millis());

  // wdt_reset();
}

void updateFSM(int direction, COLOR color, int time) {
  if (currState == WAIT_FOR_INPUT) {
    if ((color == RED) && (time - LAST_RED > RED_THRESHOLD)) {
      commandMotor(direction, color);
      LAST_RED = time;      
      currState = STOP;
      return;
    }
    if ((color != RED) || ((color == RED) && (time - LAST_RED <= RED_THRESHOLD))) {
      CURR_DIR = direction;
      CURR_COLOR = color;
      currState = MOTOR_COMMAND;
      return;
    }
    return;
  }

  if (currState == MOTOR_COMMAND) {
    commandMotor(CURR_DIR, CURR_COLOR);
    currState = WAIT_FOR_INPUT;
    return;
  }

  if (currState == STOP) {
    currState = WAIT_AFTER_STOP;
    return;
  }

  if (currState == WAIT_AFTER_STOP) {
    if (time - LAST_RED > RED_STOP_TIME) {
      currState = WAIT_FOR_INPUT;
      return;
    }
    return;
  }
}

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
  LAST_RED = millis();
  CURR_DIR = 0x0;
  CURR_COLOR = NONE;
}

COLOR pollColorSensor() {
  // Setting RED (R) filtered photodiodes to be read
  digitalWrite(COLOR_S2, LOW);
  digitalWrite(COLOR_S3, LOW);
  
  // Reading the output frequency
  int redFrequency = pulseIn(COLOR_OUT, LOW);
  delay(50);
  
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(COLOR_S2, HIGH);
  digitalWrite(COLOR_S3, HIGH);
  
  // Reading the output frequency
  int greenFrequency = pulseIn(COLOR_OUT, LOW);
  delay(50);
 
  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(COLOR_S2, LOW);
  digitalWrite(COLOR_S3, HIGH);
  
  // Reading the output frequency
  int blueFrequency = pulseIn(COLOR_OUT, LOW);

  // TODO: threshold colors and return corresponding enum
  return NONE;
}

void commandMotor(char direction, COLOR color) {
  // if no button is pressed or color is red or both forward and back are pressed then turn motors off
  if (direction == 0x0 || color == RED || ((direction & F) && (direction & B)) || !((direction & F) || (direction & B))) {
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
    return;
  }

  // set motor speed based on color
  int motorSpeed = MOTOR_DEFAULT_SPEED;
  if (color == GREEN) {
    motorSpeed = MOTOR_BOOST_SPEED;
  }
  if (color == BLUE) {
    motorSpeed = MOTOR_SLOW_SPEED;
  }

  analogWrite(LEFT_MOTOR_PWM, motorSpeed);
  analogWrite(RIGHT_MOTOR_PWM, motorSpeed);

  // set motors to move forward
  if (direction & F) {
    digitalWrite(LEFT_MOTOR_PIN1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);   
  }

  // set motors to move backward
  if (direction & B) {
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
  }

  // if both left and right are pressed don't change direction
  if ((direction & R) && (direction & L)) return;

  // if right pressed then turn off right motors
  if (direction & R) {
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  }

  // if left pressed then turn off left motors
  if (direction & L) {
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
  }
}
