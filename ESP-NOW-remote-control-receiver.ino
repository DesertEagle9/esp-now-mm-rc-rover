/*
  Rui Santos
  Complete project details at: 
  https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in 
  all copies or substantial portions of the Software.

  Royce Danam 
  Modified original code by Rui Santos to implement an ESP32 receiver to 
  receive movement commands transmitted wirelessly via ESP-NOW from one or 
  more ESP32 remote controllers. The ESP32 receiver is connected to a 
  Micromelon Rover via UART and relays the commands received by the ESP32 
  receiver via ESP-NOW to Micromelon Rover where they will be handled and 
  executed
*/

#include <esp_now.h>
#include <WiFi.h>

#define RXD_PIN 16
#define TXD_PIN 17
#define COMMAND_SIZE 11
#define EXP_MODE_SIZE 5
#define NO_ID 0

/*****************************************************************************/
/*                             DATA STRUCTURES                               */
/*****************************************************************************/
/**
 * Types of movement commands that can be received by ESP32 receiver from other
 * ESP32 remote controllers
 */
enum ControllerCommands {
  STOP,
  MOVE_FORWARD,
  MOVE_BACKWARD,
  PIVOT_LEFT,
  PIVOT_RIGHT
};

/**
 * Structure type to store controller message parameters sent by the ESP32 
 * remote control to the ESP32 receiver. The controller message contains a 
 * unique controllerId to identify each ESP32 remote control and a moveCommand 
 * to be relayed by the ESP32 receiver to the Micromelon Rover for handling 
 * and execution
 */
typedef struct ControllerMessage {
  uint8_t controllerId;
  uint8_t moveCommand;
} ControllerMessage;

/*****************************************************************************/
/*                             GLOBAL VARIABLES                              */
/*****************************************************************************/
// Default command to stop rover movement
ControllerMessage stopCommand = {NO_ID, STOP}; 
// Structure to store message received by the ESP32 receiver from an ESP32 
// remote control via ESP-NOW
ControllerMessage receivedMessage = stopCommand; 
// Structure to store message that must be relayed by the ESP32 receiver to 
// the Micromelon Rover for handling and execution
ControllerMessage relayMessage = stopCommand;

// Valid movement commands that can be relayed by the ESP32 receiver to the 
// Micromelon Rover for handling and execution
uint8_t setExpansionMode[] = {85, 1, 26, 1, 1};
uint8_t moveForwardPacket[] = {85, 1, 0, 7, 42, 42, 0, 0, 0, 0, 0}; 
uint8_t moveBackwardPacket[] = {85, 1, 0, 7, 214, 214, 0, 0, 0, 0, 0}; 
uint8_t pivotLeftPacket[] = {85, 1, 0, 7, 214, 42, 0, 0, 0, 0, 0}; 
uint8_t pivotRightPacket[] = {85, 1, 0, 7, 42, 214, 0, 0, 0, 0, 0}; 
uint8_t stopPacket[] = {85, 1, 0, 7, 0, 0, 0, 0, 0, 0, 0}; 


/*****************************************************************************/
/*                                 FUNCTIONS                                 */
/*****************************************************************************/
/**
 * Writes the movement command stored in the relayMessage structure to the 
 * serial port of the ESP32 receiver for transmission to the Micromelon Rover 
 * over UART
 */
void transmit_command() {
  switch (relayMessage.moveCommand) {
    case MOVE_FORWARD:
      Serial2.write(moveForwardPacket, COMMAND_SIZE);
      Serial.println("moveForwardPacket");
      break;
    case MOVE_BACKWARD:
      Serial2.write(moveBackwardPacket, COMMAND_SIZE);
      Serial.println("moveBackwardPacket");
      break;
    case PIVOT_LEFT:
      Serial2.write(pivotRightPacket, COMMAND_SIZE);
      Serial.println("pivotRightPacket");
      break;
    case PIVOT_RIGHT:
      Serial2.write(pivotLeftPacket, COMMAND_SIZE);
      Serial.println("pivotLeftPacket");
      break;
    default:
      Serial2.write(stopPacket, COMMAND_SIZE);
      Serial.println("stopPacket");
      break;
  }
}

/**
 * Relays the received message from one or more ESP32 remote controllers to
 * to the Micromelon Rover via UART
 */
void relay_message() {
  // Relay a stopCommand to the Micromelon Rover
  if (receivedMessage.controllerId != NO_ID &&
      receivedMessage.moveCommand == STOP && 
      relayMessage.controllerId == receivedMessage.controllerId) {
    relayMessage = stopCommand;
    transmit_command();
  } 
  // Relay a message containing a new command to the Micromelon Rover
  if (relayMessage.controllerId == NO_ID &&
      receivedMessage.moveCommand != STOP) {
    memcpy(&relayMessage, &receivedMessage, sizeof(relayMessage));
    transmit_command();
  }
}

/**
 * ESP-NOW callback function that is executed when the ESP32 receiver
 * is notified of a new message sent by a ESP32 remote controller
 */
void receive_new_message(const uint8_t* macAddress, 
    const uint8_t* incomingMessage, int size) {
  memcpy(&receivedMessage, incomingMessage, sizeof(receivedMessage));
  relay_message();
}


/*****************************************************************************/
/*                                 MAIN LOOP                                 */
/*****************************************************************************/
void setup() {
  // Begin serial monitor for error messages
  Serial.begin(115200); 
  // Begin serial port to transmit messages from ESP32 to Micromelon Rover
  Serial2.begin(115200, SERIAL_8N1, TXD_PIN, RXD_PIN); 
  // Set Micromelon Rover to Expansion Mode by transmitting the corresponding 
  // via the serial connection
  Serial2.write(setExpansionMode, EXP_MODE_SIZE);

  // Set ESP32 receiver as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialise ESP-NOW protocol
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESP-NOW is successfully initialised, we will register for recv CB to
  // get recv packet info
  esp_now_register_recv_cb(receive_new_message);
}

void loop() {
  ;
}