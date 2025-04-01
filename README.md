Project: Smart Waste Sorting System
📌 Overview
This project implements a smart waste classification system using an Arduino Nano 33 BLE Sense with an OV7670 camera, a Raspberry Pi (RPi) for inference, and Bluetooth communication between them.

When an object is detected via a touch sensor, the Arduino captures an image, sends it to the RPi via Bluetooth (using BluPy), and the RPi runs a pre-trained CNN model (model.h5) to classify the object into one of six categories:

cardboard

metal

paper

plastic

trash

white-glass

After classification, the RPi activates a motor (via GPIO) to sort the waste and prints the detected class in the terminal.

🛠 Hardware Components
Arduino Nano 33 BLE Sense (with BLE support)

OV7670 Camera Module (for image capture)

Capacitive Touch Sensor (to detect object presence)

Raspberry Pi (for running the ML model)

Servo Motor / Stepper Motor (for waste sorting)

Jumper Wires & Breadboard (for connections)

📂 Repository Structure

Smart-Waste-Sorter/  
├── Arduino/  
│   ├── waste_sorter_ble.ino       # Arduino BLE & Camera Code  
│   └── ov7670_config.h            # OV7670 Camera Settings  
├── RaspberryPi/  
│   ├── bluetooth_receiver.py      # BluPy Bluetooth Receiver  
│   ├── classify_waste.py          # CNN Model Inference  
│   └── model.h5                   # Pre-trained Keras Model  
├── Docs/  
│   ├── wiring_diagram.png         # Hardware Connections  
│   └── demo_video.mp4             # System Demo  
└── README.md                      # This File  

⚙️ How It Works
1️⃣ Arduino Nano 33 BLE Sense (Sender)
Touch Sensor Detection:

When an object is placed near the sensor, it triggers the camera.

OV7670 Camera Capture:

Captures a 320x240 grayscale image (optimized for BLE transfer).

Bluetooth Low Energy (BLE) Transmission:

Creates a custom BLE service (BLE_SERVICE_UUID) and sends the image data.

2️⃣ Raspberry Pi (Receiver & Classifier)
Bluetooth Data Reception (bluetooth_receiver.py):

Uses BluPy to receive the image from Arduino.

CNN Model Inference (classify_waste.py):

Loads the pre-trained model.h5 (Keras/TensorFlow).

Classifies the image into one of 6 waste categories.

Motor Control & Output:

Activates a GPIO-controlled motor to sort the waste.

Prints the detected class (e.g., "Detected: plastic").

🔌 Wiring Guide
Arduino Nano 33 BLE Sense
OV7670 Pin	Arduino Pin
VCC	3.3V
GND	GND
SIOC	SCL
SIOD	SDA
VSYNC	D9
HREF	D8
PCLK	D11
XCLK	D3
D0-D7	D4-D10
Touch Sensor → A0 (Analog Input)

Raspberry Pi
Motor → GPIO17 (or any PWM-capable pin)

Bluetooth → Built-in (using BluPy library)

🚀 Setup & Execution
1. Flash Arduino Code
Upload waste_sorter_ble.ino to the Arduino Nano 33 BLE Sense.

Ensure ov7670_config.h is properly configured.

2. Raspberry Pi Setup
# Install dependencies  
pip install tensorflow numpy blupy RPi.GPIO  

# Run the receiver & classifier  
python3 bluetooth_receiver.py  
python3 classify_waste.py  

3. Test the System
Place an object near the touch sensor.

Arduino captures & sends the image via BLE.

RPi classifies it and prints the result.

Motor moves to sort the waste.

📊 Expected Output

[Bluetooth] Image received from Arduino.  
[Model] Loading model.h5...  
[Prediction] Detected: metal (92% confidence)  
[GPIO] Activating motor for metal sorting...  

📜 License
MIT License - Free for personal & educational use.
