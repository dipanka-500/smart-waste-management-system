
#include <Arduino_OV767X.h>
#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>
#include <Arduino_HS300x.h>

long bytesPerFrame; // variable to hold total number of bytes in image
long numPixels;
 
// Define the UUID for the service and characteristics
#define BLE_SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define IMAGE_CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"


//for the touch sensor
#define BUTTON_PIN1 11
struct touch { 
	 byte wasPressed = LOW; 
	 byte isPressed = LOW; 
}; 
touch touch1; 

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

// Variables to store ML prediction results
byte predictionClass = 0;
float predictionConfidence = 0.0;
bool newPredictionReceived = false;
bool isTouchPressed(int pin) 
{ 
	 return digitalRead(pin) == HIGH; 
} 


void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("=== Arduino Nano 33 BLE - Image Capture with ML ===");
  pinMode(BUTTON_PIN1, INPUT);
  // Begin the OV7670 specifying resolution (QCIF, Pixel format RGB565 and frames per second)
  if (!Camera.begin(QCIF, RGB565, 1)) {
    Serial.println("Failed to initialize camera!");
    while (1);
  }
  Serial.println("Camera initialized successfully");
  
  bytesPerFrame = Camera.width() * Camera.height() * Camera.bytesPerPixel();
  numPixels = Camera.width() * Camera.height();
 
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
  
  BLE.addService(bleService); // Add service

     
  BLE.advertise(); // Start advertising
  Serial.print("Peripheral device MAC: ");
  Serial.println(BLE.address());
  Serial.println("Waiting for connections...");
}



void loop() {
  BLEDevice central = BLE.central(); // Wait for a BLE central to connect
  
  // If central is connected to peripheral
  if (central) {
      Serial.print("Connected to central MAC: ");
      Serial.println(central.address()); // Central's BT address:
      digitalWrite(LED_BUILTIN, HIGH); // Turn on the LED to indicate the connection
      
      Serial.println("waitting for a object");
      
      while (central.connected()) {
        // Process BLE events
        BLE.poll();
        
        
        
        // Wait for a 'c' from Serial port before taking frame
        Serial.println("waitting fora object to take image");
        touch1.isPressed = isTouchPressed(BUTTON_PIN1);

    if (touch1.wasPressed != touch1.isPressed) { 
      Serial.println("Touch pressed"); 
          Serial.println("\nCapturing image...");
         
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
