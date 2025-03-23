📦 Smart Waste Management System
📝 Overview
This project implements an AI-powered Smart Waste Management System using Arduino Nano 33 BLE Sense, Raspberry Pi, a touch sensor, and a conveyor belt. The system captures an image of waste, classifies it, and moves the conveyor belt based on the predicted category.

🔧 Hardware Components
Arduino Nano 33 BLE Sense (for capturing images & Bluetooth communication)

Raspberry Pi (for running the AI model & prediction)

Touch Sensor (to detect object presence before taking a photo)

Motor Driver (to control the conveyor belt)

DC Motor (for moving the conveyor belt)

🖥️ Software & Dependencies
Arduino IDE (for programming Arduino Nano 33 BLE Sense)

Python & OpenCV (for image processing)

TensorFlow / PyTorch (for waste classification model)

Bluetooth Communication (for Arduino-Raspberry Pi data transfer)

⚙️ Working Principle
Touch Sensor Activation: The system detects if an object is present before capturing an image.

Image Capture: The Arduino Nano 33 BLE Sense captures an image of the waste.

Bluetooth Communication: The image is sent to the Raspberry Pi via Bluetooth.

AI Model Prediction: The Raspberry Pi runs a machine learning model to classify the waste.

Feedback to Arduino: The predicted waste category (e.g., Plastic, Organic, Metal) is sent back to the Arduino.

Conveyor Belt Movement: Based on the classification, the conveyor belt moves to sort the waste correctly.
