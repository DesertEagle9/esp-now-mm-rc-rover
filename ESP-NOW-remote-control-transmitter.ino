/*
  Rui Santos
  Complete project details at: 
  https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in 
  all copies or substantial portions of the Software.

  Royce Danam 
  Modified original code by Rui Santos to implement an ESP32 remote controller 
  that wirelessly transmits movement commands via ESP-NOW to an ESP32 receiver 
  connected to a Micromelon Rover via UART
  
  Instructions to get ESP MAC Address: 
  https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/
*/

#include <esp_now.h>
#include <ezButton.h>
#include <WiFi.h>

#define MAC_ADDR_SIZE 6 
#define BUTTON_PIN 22

/*****************************************************************************/
/*                             DATA STRUCTURES                               */
/*****************************************************************************/
/**
 * Types of movement commands that can be sent from the ESP32 remote control
 */
enum ControllerCommands {
  STOP,
  MOVE_FORWARD,
  MOVE_BACKWARD,
  PIVOT_LEFT,
  PIVOT_RIGHT
};

/** 
 * Unique ID that can be assigned to multiple ESP32 remote controls
 */
enum ControllerId {
  CONTROLLER_A = 1,
  CONTROLLER_B,
  CONTROLLER_C,
  CONTROLLER_D
};

/**
 * Structure type to store controller message parameters sent by the ESP32 
 * remote control to the ESP32 receiver. The controller message contains a 
 * unique controllerId to identify each ESP32 remote control and a moveCommand 
 * to be relayed by the ESP32 receiver to the Micromelon Rover for handling 
 * and execution
 */
struct ControllerMessage {
  uint8_t controllerId;
  uint8_t moveCommand;
} controllerMessage = {CONTROLLER_B, STOP}; 


/*****************************************************************************/
/*                             GLOBAL VARIABLES                              */
/*****************************************************************************/
// MAC address of ESP32 receiver connected to the Micromelon Rover via UART
uint8_t broadcastAddress[] = {0x80, 0x7D, 0x3A, 0xB9, 0xCC, 0xCC};
// Data type for ESP-NOW protocol
esp_now_peer_info_t peerInfo;
// Initialise button type to use ezButton library
ezButton button(BUTTON_PIN);


/*****************************************************************************/
/*                                 FUNCTIONS                                 */
/*****************************************************************************/
/**
 * Standard ESP-NOW callback function to indicate whether data was successfully
 * sent via the protocol
 */
void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? 
    "Delivery Success" : "Delivery Fail");
}

/**
 * Initialise the device to use the ESP-NOW protocol to be used by the remote 
 * control to transmit movement command packets 
 */
void setup_esp_now() {
  // Setup device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialise ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESP-NOW is successfully initialised, we will register for send 
  // callback to get the status of the transmitted packet
  esp_now_register_send_cb(on_data_sent);
  
  // Register ESP32 receiver
  memcpy(peerInfo.peer_addr, broadcastAddress, MAC_ADDR_SIZE);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add ESP32 receiver        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

/**
 * Transmit corresponding movement commands to the ESP32 receiver via ESP-NOW 
 * whenever the user presses or releases a button on the ESP32 remote control
 */
void transmit_command() {
  esp_err_t result;

  // Send a movement command when the button on the remote control is pressed
  if (button.isPressed()) {
    // Send move command
    controllerMessage.moveCommand = MOVE_BACKWARD;
    result = esp_now_send(broadcastAddress, 
      (uint8_t *) &controllerMessage, sizeof(controllerMessage));
    
    // Check if command was sent successfully
    Serial.println(result == ESP_OK ? 
      "Sent with success" : "Error sending move command");
    
    // Wait for the button to be released before sending a stop command
    while (1) {
      button.loop(); 
      if (button.isReleased()) {
        // Send stop command
        controllerMessage.moveCommand = STOP;
        result = esp_now_send(broadcastAddress, 
          (uint8_t *) &controllerMessage, sizeof(controllerMessage));
        
        // Check if command was sent successfully
        Serial.println(result == ESP_OK ? 
          "Sent with success" : "Error sending stop command");
        break; // Exit infinite while loop
      }
    }
  }
}

/*****************************************************************************/
/*                                 MAIN LOOP                                 */
/*****************************************************************************/
void setup() {
  Serial.begin(115200);
  button.setDebounceTime(50); 
  setup_esp_now();
}

void loop() {
  button.loop();
  transmit_command();
}
