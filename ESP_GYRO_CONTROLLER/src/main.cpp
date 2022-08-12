/* ==========================================================
   ------------------------ CONTROLLER --------------------------
   ==========================================================

   Controller MAC: 24:0A:C4:EC:0D:B4
   Peripheral MAC: 24:0A:C4:EC:A6:F0

   ** CONTROLLER RECEIVES ACCELEROMETER DATA FROM PERIPHERAL AND PRINTS TO SERIAL **
*/

#include <Arduino.h>
#include <esp_now.h>
#include <Wifi.h>
#include <SPI.h>
#include <iostream>
#include <stdlib.h>
using namespace std;

bool system_initialized = false;

const int SOLENOID_PIN = 16; // pin for the solenoid

//uint8_t controllerBroadcastAddress[] = { 0x24, 0x0A, 0xC4, 0xEC, 0x07, 0xCC }; // old?
uint8_t controllerBroadcastAddress[] = { 0x24, 0x0A, 0xC4, 0xEC, 0x0D, 0xB4 };
uint8_t peripheralBroadcastAddresss[] = { 0x24, 0x0A, 0xC4, 0xEC, 0xA6, 0xF0 };

// these variables are for holding any received data
float incoming_gyro_yaw;
float incoming_gyro_pitch;
float incoming_gyro_roll;

typedef struct struct_accel_message {
  float gyro_yaw;
  float gyro_pitch;
  float gyro_roll;
  unsigned long total_measurement_time; // total time it took for 1 measurement (milliseconds)
} struct_accel_message;

//struct_accel_message incomingSensorReading; // for controller code... old.. moved to OnDataReceive
                                              // to hopefully make memory cleanup and garbage collection better
struct_accel_message outgoingSensorReading; // for peripheral code

esp_now_peer_info_t peerInfo;

unsigned long last_message_received_time = 0;

// These variables are for helping us track the time between each measurement
unsigned long count = 0;
unsigned long total_time_delta = 0;

void printAccelerometerDataNice() {
  //if(millis() - last_message_received_time >= 1000) {
    //count = 0;
    // i believe there are memory leak problems so let's keep track of the memory usage
    //Serial.print(" --> DEBUG --> Total Free Heap.. "); Serial.print(ESP.getFreeHeap());    Serial.println(" bytes");
    //Serial.print(" --> DEBUG --> Min Free Heap.... "); Serial.print(ESP.getMinFreeHeap()); Serial.println(" bytes");
  //} 

  //Serial.println("I'm receiving the data, don't worry about it.");
  // Serial.print("ypr\t");
  // Serial.print(incoming_gyro_yaw);
  // Serial.print("\t");
  // Serial.print(incoming_gyro_pitch);
  // Serial.print("\t");
  // Serial.println(incoming_gyro_roll);

  //last_message_received_time = millis();
}

//unsigned long maximum = 0;
void activateSolenoid() {
  int number = random(1, 11);
  Serial.println(number);
  if(number == 5) {
    Serial.println("random number was 5");
    return;
  }
  else {
    digitalWrite(SOLENOID_PIN, HIGH);
    delay(100); //gives Arduino time to look for this
    digitalWrite(SOLENOID_PIN, LOW);
  }
}

// when this microcontroller sends a message, this function is triggered
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\nLast Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success":"Delivery Failure");
}

// when this microcontroller receives a message, this function is triggered
float previousGyro = 0;
float twoGyrosAgo = 0;
void OnDataReceive(const uint8_t* mac_addr, const uint8_t *incomingData, int len) {
  struct_accel_message incomingSensorReading; // hopefully this memory will be destroyed 
                                              // now upon exiting this function
  memcpy(&incomingSensorReading, incomingData, sizeof(incomingSensorReading));

  // now that the incomingSensorReading memory structure's values are filled  
  // in, move the data into local variables so we can do stuff with it
  incoming_gyro_yaw = incomingSensorReading.gyro_yaw;
  incoming_gyro_pitch = incomingSensorReading.gyro_pitch;
  incoming_gyro_roll = incomingSensorReading.gyro_roll;

  //printAccelerometerDataNice();

  total_time_delta            = incomingSensorReading.total_measurement_time;

  // we've harvested all the data from the incoming data packet and stored 
  // it into local variables. 
  // Now print it nicely to the console so we can see it
  //printAccelerometerDataNice();
  if(!system_initialized) {
    previousGyro = incoming_gyro_yaw;
    system_initialized = true;
    return;
  }
  
  if(((twoGyrosAgo - previousGyro) >= 0.008) && ((incoming_gyro_yaw - previousGyro) >= 0.008)) {
    Serial.print("Two Gyros Ago:");
    Serial.println(twoGyrosAgo);
    Serial.print("Previous:");
    Serial.println(previousGyro);
    Serial.print("Incoming:");
    Serial.println(incoming_gyro_yaw);
    activateSolenoid();
    //delay(5000); // to stop activating the solenoid multiple times???
  }
  else {
    //Serial.println("Condition was not met.");
  }
  
  twoGyrosAgo = previousGyro;
  previousGyro = incoming_gyro_yaw;
}

void setup() {
  // 
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); // station
  Serial.print("Hello, i'm Controller, my MAC Address is: ");
  Serial.println(WiFi.macAddress());
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);

  if(esp_now_init() != ESP_OK) {
    Serial.println("Error Initializing ESP-NOW!");
    for(;;); // just loop forever because there's no point going further
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, peripheralBroadcastAddresss, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if(esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    for(;;); // just lay down and die
  }

  esp_now_register_recv_cb(OnDataReceive);

  Serial.println("...WAITING FOR DATA\n");
}

// controller
void loop() {
  // controller won't have anything in its loop. It just sits and waits to receive data
}