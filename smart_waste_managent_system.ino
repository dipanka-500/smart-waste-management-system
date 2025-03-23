#include "mbed.h"
#include <Arduino_OV767X.h>
#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>
#include <Arduino_HS300x.h>

long bytesPerFrame; // variable to hold total number of bytes in image
long numPixels;
 
// Define the UUID for the service and characteristics
#define BLE_SERVICE_UUID "12345678-1234-5678-1234-56789abcdef1"
#define IMAGE_CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef2"
#define RESULT_CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef3"
//for touch sensor
  #define BUTTON_PIN 4 
  //motor driver definition of pin configuration

#define input1=2;
#define input2=3;

#define enablePin1=5;
//for the touch sensor 
  #define BUTTON_PIN1 4 
// Create a BLE service and characteristics
BLEService bleService(BLE_SERVICE_UUID);

// Declare a byte array to hold the raw pixels received from the OV7670
// Array size is set for QCIF; if other format required, change size
// QCIF: 176x144 X 2 bytes per pixel (RGB565)
byte data[176 * 144 * 2]; 

// Maximum packet size for BLE transfer (MTU size - 3 bytes for ATT header)
const int MAX_PACKET_SIZE = 20;

// Characteristics for image data and ML results
BLECharacteristic imageCharacteristic(IMAGE_CHARACTERISTIC_UUID, BLERead | BLENotify, MAX_PACKET_SIZE);
BLECharacteristic resultCharacteristic(RESULT_CHARACTERISTIC_UUID, BLERead | BLEWrite | BLENotify, sizeof(byte) + sizeof(float)); // Class index (1 byte) + confidence (4 bytes)
// for touch sensor 
struct touch { 
	 byte wasPressed = LOW; 
	 byte isPressed = LOW; 
}; 
// Variables to store ML prediction results
byte predictionClass = 0;
float predictionConfidence = 0.0;
bool newPredictionReceived = false;

// Class labels for the model (update these with your model's classes)
const char* classLabels[] = {
  "Class 0",
  "Class 1", 
  "Class 2",
  "Class 3",
  "Class 4",
  "Class 5",
  "Class 6",
  
};
const int numClasses = 6;
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("=== Arduino Nano 33 BLE - Image Capture with ML ===");
  
  // Begin the OV7670 specifying resolution (QCIF, Pixel format RGB565 and frames per second)
  if (!Camera.begin(QCIF, RGB565, 1)) {
    Serial.println("Failed to initialize camera!");
    while (1);
  }
  Serial.println("Camera initialized successfully");
  
  bytesPerFrame = Camera.width() * Camera.height() * Camera.bytesPerPixel();
  numPixels = Camera.width() * Camera.height();
  pinMode(BUTTON_PIN1, INPUT);
  pinMode(input1,OUTPUT);
  pinMode(input2,OUTPUT);
   pinMode(enablePin1,OUTPUT);
  Serial.print("Image resolution: ");
  Serial.print(Camera.width());
  Serial.print("x");
  Serial.print(Camera.height());
  Serial.print(", ");
  Serial.print(bytesPerFrame);
  Serial.println(" bytes per frame");
   
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the built-in LED pin
   
  if (!BLE.begin()) { // Initialize NINA B306 BLE
      Serial.println("Starting BLE failed!");
      while (1);
  }
  
  Serial.println("BLE initialized successfully");
  
  BLE.setLocalName("JH_ArduinoNano33BLESense_R2");    // Set name for connection
  BLE.setAdvertisedService(bleService); // Advertise ble service
   
  bleService.addCharacteristic(imageCharacteristic);    // Add image characteristic
  bleService.addCharacteristic(resultCharacteristic);   // Add result characteristic
 
  BLE.addService(bleService); // Add service

  // Set up callback for when prediction results are received
  resultCharacteristic.setEventHandler(BLEWritten, onResultReceived);
     
  BLE.advertise(); // Start advertising
  Serial.print("Peripheral device MAC: ");
  Serial.println(BLE.address());
  Serial.println("Waiting for connections...");
}

void onResultReceived(BLEDevice central, BLECharacteristic characteristic) {
  // Read the prediction result sent from Raspberry Pi
  if (characteristic.uuid() == RESULT_CHARACTERISTIC_UUID) {
    const byte* data = characteristic.value();
    
    // Extract class index (first byte)
    predictionClass = data[0]; 
    
    // Convert 4 bytes to float (assumes little-endian)
    float confidence;
    memcpy(&confidence, &data[1], sizeof(float));
    predictionConfidence = confidence;

    // Mark that we've received a new prediction
    newPredictionReceived = true;
    
    // Print formatted prediction results
    Serial.println("\n==== ML PREDICTION RESULTS ====");
    Serial.print("Class Index: ");
    Serial.println(predictionClass);
    
    // Print class label if available
    if (predictionClass < numClasses) {
      Serial.print("Class Label: ");
      Serial.println(classLabels[predictionClass]);
    } else {
      Serial.println("Warning: Unknown class index!");
    }
    
    // Print confidence with proper formatting
    Serial.print("Confidence: ");
    Serial.print(predictionConfidence * 100.0, 2); // Convert to percentage with 2 decimal places
    Serial.println("%");
    
    // Print confidence as bar graph
    Serial.print("Confidence level: [");
    int barLength = (int)(predictionConfidence * 20.0);
    for (int i = 0; i < 20; i++) {
      if (i < barLength) {
        Serial.print("█");
      } else {
        Serial.print(" ");
      }
    }
    Serial.println("]");
    Serial.println("============================");
    
    // Activate motors when prediction is received
    // Make sure the confidence is above a certain threshold (optional)
    if (predictionConfidence > 0.5) {  // Adjust threshold as needed
      // Activate the motors
      analogWrite(enablePin1, 150);
      // Note: enablePin2, input3, and input4 are not defined in your original code
      // You'll need to define these in your setup
      // analogWrite(enablePin2, 150);
      digitalWrite(input1, HIGH);
      digitalWrite(input2, HIGH);
      // digitalWrite(input3, HIGH);  
      // digitalWrite(input4, HIGH);
      
      Serial.println("Motors activated based on prediction!");
      
      // Optional: Stop motors after some time
      // delay(2000);  // Run motors for 2 seconds
      // digitalWrite(input1, LOW);
      // digitalWrite(input2, LOW);
      // analogWrite(enablePin1, 0);
    }
  }
}

void loop() {
  BLEDevice central = BLE.central(); // Wait for a BLE central to connect
  
  // If central is connected to peripheral
  if (central) {
      Serial.print("Connected to central MAC: ");
      Serial.println(central.address()); // Central's BT address:
      digitalWrite(LED_BUILTIN, HIGH); // Turn on the LED to indicate the connection
      
      Serial.println("Connection established. Press 'c' to capture image.");
      
      while (central.connected()) {
        // Process BLE events
        BLE.poll();
        
        // Display indicator if new prediction was received
        if (newPredictionReceived) {
          // Visual indicator using built-in LED
          if (predictionConfidence > 0.8) {
            // High confidence prediction - double blink
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
          } else {
            // Lower confidence - single blink
            digitalWrite(LED_BUILTIN, HIGH);
            delay(200);
            digitalWrite(LED_BUILTIN, LOW);
          }
          
          newPredictionReceived = false;
        }
        
        // Wait for a 'c' from Serial port before taking frame
        if (Serial.available() > 0 && Serial.read() == 'c') {
          Serial.println("\nCapturing image...");
          touch1.isPressed = isTouchPressed(BUTTON_PIN1);
    if (touch1.wasPressed != touch1.isPressed) { 
      Serial.println("Touch pressed"); 
          // Read frame from OV7670 into byte array
          Camera.readFrame(data);
          
          // Send the image data over BLE in chunks
          int offset = 0;
          byte packet[MAX_PACKET_SIZE];
          
          Serial.println("Sending image via BLE...");
          unsigned long startTime = millis();
          
          while (offset < bytesPerFrame) {
            // Determine packet size for this chunk
            int packetSize = min(MAX_PACKET_SIZE, bytesPerFrame - offset);
            
            // Copy data to packet buffer
            for (int i = 0; i < packetSize; i++) {
              packet[i] = data[offset + i];
            }
            
            // Send packet
            imageCharacteristic.writeValue(packet, packetSize);
            
            // Update offset
            offset += packetSize;
            
            // Print progress every ~10%
            if (offset % (bytesPerFrame / 10) < MAX_PACKET_SIZE) {
              Serial.print(".");
            }
            
            // Small delay to avoid overwhelming the BLE stack
            delay(5);
          }
          
          unsigned long transferTime = millis() - startTime;
          Serial.println();
          Serial.print("Image sent in ");
          Serial.print(transferTime);
          Serial.println(" ms");
          Serial.println("Waiting for prediction results...");
        }
        
        delay(100);
        }
      }
      
      digitalWrite(LED_BUILTIN, LOW); // When the central disconnects, turn off the LED
      Serial.print("Disconnected from central MAC: ");
      Serial.println(central.address());
      Serial.println("Waiting for new connection...");
  }
}