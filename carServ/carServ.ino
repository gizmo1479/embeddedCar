#include <SPI.h>
#include <WiFi101.h>
#include "common.h"

#define S_SSID "Kal" // name of network trying to connect to
#define SPASS "R3slif3sux42069"
char ssid[] = S_SSID;        // your network SSID (name)
char pass[] = SPASS;
int status = WL_IDLE_STATUS;     // the WiFi server's status
WiFiServer server(8888); // Our server on port 8888

int FORWARD_PIN = 7; // button pin to move forward
int BACKWARD_PIN = 8; // button pin to move backward
int LEFT_PIN = 9;
int RIGHT_PIN = 10;

void setup() {
  //Initialize serial and wait for port to open:
  initPins();
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to open SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // begin accepting clients and connect to the server
  startServer();
}

void loop() {
  // check for client every 500 ms
  delay(500);
  WiFiClient client = server.available();
  DIR d = pollPins();
  Serial.println(d);
  server.write(d);
//  if (client) {
//    // TODO: should there be a while loop here instead??
//    if (client.connected()) {
//      DIR d = pollPins();
//      Serial.print("Connected to client, sending button input of: ");
//      Serial.println(d);
//      
//    } else {
//      Serial.println("im not sure how this works but the client isn't connected???, so stopping");
//      client.stop();
//    }
//    
//  } else {
//    Serial.println("no client found, trying again in 5 seconds...");
//    delay(5000);
//  }
}

void initPins() {
  pinMode(FORWARD_PIN, INPUT);
  pinMode(BACKWARD_PIN, INPUT);
  pinMode(LEFT_PIN, INPUT);
  pinMode(RIGHT_PIN, INPUT);
}

void startServer() {
  Serial.println("Connected to the network");
  server.begin();
  Serial.print("Starting server, IP: ");
  IPAddress myAddr = WiFi.localIP();
  Serial.println(myAddr);
  Serial.println("Now accepting clients");
}

DIR pollPins() {
    if (digitalRead(FORWARD_PIN)) {
      return FORWARD;
    }
//    } else if (digitalRead(BACKWARD_PIN)) {
//      return BACKWARD;
//    } else if (digitalRead(LEFT_PIN)) {
//      return LEFT;
//    } else if (digitalRead(RIGHT_PIN)) {
//      return RIGHT;
//    }

    return NONE;
}
