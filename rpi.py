from bluepy import btle
import time
import struct
import numpy as np
from PIL import Image
import os
import ctypes
import RPi.GPIO as GPIO

# BLE device information
mac_address = "41:22:3b:44:57:6c"  # Put your Arduino's MAC address here
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef1"  # Service UUID from Arduino code
IMAGE_CHARACTERISTIC_UUID = "12345678-1234-5678-1234-56789abcdef2"  # Image characteristic UUID

# Image parameters (must match camera settings on Arduino)
WIDTH = 176  # QCIF width
HEIGHT = 144  # QCIF height
BYTES_PER_PIXEL = 2  # RGB565 format
BYTES_PER_FRAME = WIDTH * HEIGHT * BYTES_PER_PIXEL

# Model settings
MODEL_PATH = "/home/pipi/Desktop/smart_waste_management/model (1).h5" 
INPUT_SIZE = (224, 224)  # Model input size, adjust as needed
CLASSES = ["cardboard", "metal", "paper", "plastic", "trash", "white-glass"]

# Motor Control GPIO Pins
# Right Motor
in1 = 17
in2 = 27
en_a = 4

# Left Motor
in3 = 5
in4 = 6
en_b = 13

# GPIO Setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(en_a, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(en_b, GPIO.OUT)

# PWM Setup
q = GPIO.PWM(en_a, 100)
p = GPIO.PWM(en_b, 100)
p.start(75)
q.start(75)

# Initial Motor Stop
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)
GPIO.output(in3, GPIO.LOW)

class NotificationDelegate(btle.DefaultDelegate):
    def __init__(self):
        btle.DefaultDelegate.__init__(self)
        self.image_buffer = bytearray()
        self.frame_complete = False
        self.total_bytes_received = 0
        
    def handleNotification(self, cHandle, data):
        # Add received data to our buffer
        self.image_buffer.extend(data)
        self.total_bytes_received += len(data)
        
        # Check if we've received the entire frame
        if self.total_bytes_received >= BYTES_PER_FRAME:
            self.frame_complete = True
            print(f"Frame complete! Received {self.total_bytes_received} bytes")

def connect_to_device(mac_address):
    try:
        print(f"Connecting to {mac_address}...")
        device = btle.Peripheral(mac_address)
        print("Connected!")
        return device
    except btle.BTLEDisconnectError:
        print("Failed to connect. Retrying in 3 seconds...")
        time.sleep(3)
        return connect_to_device(mac_address)

def rgb565_to_rgb888(rgb565_bytes):
    """Convert RGB565 bytes to RGB888 format"""
    rgb888_data = bytearray(WIDTH * HEIGHT * 3)
    
    for i in range(0, len(rgb565_bytes), 2):
        if i+1 >= len(rgb565_bytes):
            break
            
        # Extract 16-bit RGB565 value (little endian)
        rgb565 = rgb565_bytes[i] | (rgb565_bytes[i+1] << 8)
        
        # Extract RGB components
        r = (rgb565 & 0xF800) >> 11
        g = (rgb565 & 0x07E0) >> 5
        b = rgb565 & 0x001F
        
        # Convert to RGB888
        r8 = (r * 255 // 31)
        g8 = (g * 255 // 63)
        b8 = (b * 255 // 31)
        
        # Write to RGB888 array (3 bytes per pixel)
        idx = (i // 2) * 3
        rgb888_data[idx] = r8
        rgb888_data[idx + 1] = g8
        rgb888_data[idx + 2] = b8
        
    return rgb888_data

def load_modelh_model(model_path):
    """Load model from model.h file"""
    try:
        # Extract the model data from the .h file
        model_array = []
        with open(model_path, 'r') as f:
            lines = f.readlines()
            
            # Look for the array data in the file
            for line in lines:
                line = line.strip()
                # Skip comments, preprocessor directives, etc.
                if line.startswith('//') or line.startswith('#') or not line:
                    continue
                    
                # Extract hex values
                if '0x' in line:
                    values = [int(val.strip(), 16) for val in line.replace('{', '').replace('}', '').replace(',', ' ').split() if '0x' in val]
                    model_array.extend(values)
        
        print(f"Loaded model with {len(model_array)} bytes")
        
        # For this example, we'll use a simple class to mimic the TFLite interpreter
        # In a real implementation, you would use the appropriate inference engine for your model type
        class ModelInterpreter:
            def __init__(self, model_data):
                self.model_data = model_data
                self.input_shape = [1, INPUT_SIZE[0], INPUT_SIZE[1], 3]  # Batch, Height, Width, Channels
                self.output_shape = [1, len(CLASSES)]  # Batch, Classes
                print(f"Model loaded. Input shape: {self.input_shape}, Output shape: {self.output_shape}")
            
            def predict(self, input_data):
                """
                Simple placeholder for model prediction.
                In a real implementation, you would use the appropriate inference engine.
                """
                print("Running inference with custom model.h model")
                # This is where you would actually run your model
                # For demonstration, we're just returning random predictions
                return np.random.rand(1, len(CLASSES))
        
        interpreter = ModelInterpreter(model_array)
        
        # Create a simple input/output details structure similar to TFLite
        input_details = [{'shape': interpreter.input_shape}]
        output_details = [{'shape': interpreter.output_shape}]
        
        return interpreter, input_details, output_details
    except Exception as e:
        print(f"Error loading model: {e}")
        return None, None, None

def preprocess_image(image, target_size):
    """Preprocess image for model inference"""
    # Resize image to target size
    image = image.resize(target_size)
    
    # Convert to numpy array and normalize
    img_array = np.array(image, dtype=np.float32)
    
    # Normalize to [0,1]
    img_array = img_array / 255.0
    
    # Add batch dimension
    img_array = np.expand_dims(img_array, axis=0)
    
    return img_array

def run_inference(interpreter, input_details, output_details, image):
    """Run inference on preprocessed image"""
    # In our custom implementation, we just call the predict method directly
    output_data = interpreter.predict(image)
    return output_data

def decode_prediction(prediction, top=1):
    """Convert model output to class prediction"""
    # For simplicity, assuming prediction is a probabilities array
    # You might need to modify this based on your model's output format
    top_indices = np.argsort(prediction[0])[-top:][::-1]
    results = [
        (CLASSES[idx], float(prediction[0][idx]))
        for idx in top_indices
    ]
    return results

def move_motor_forward():
    """Move motor forward"""
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)
    print("Moving Forward")
    time.sleep(2)  # Move forward for 2 seconds
    
    # Stop motors
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    print("Stopped")

def main():
    frame_count = 0
    
    # Create output directory if it doesn't exist
    os.makedirs("captured_images", exist_ok=True)
    
    # Load ML model
    print("Loading model...")
    interpreter, input_details, output_details = load_modelh_model(MODEL_PATH)
    
    if interpreter is None:
        print("Failed to load model. Exiting.")
        return
    
    try:
        # Connect to BLE device
        device = connect_to_device(mac_address)
        
        # Set up notification delegate
        delegate = NotificationDelegate()
        device.withDelegate(delegate)
        
        # Discover services and characteristics
        print("Discovering services...")
        services = device.services
        service = device.getServiceByUUID(SERVICE_UUID)
        
        print("Discovering characteristics...")
        characteristics = service.getCharacteristics()
        
        # Find the image characteristic
        image_char = None
        
        for char in characteristics:
            if char.uuid == IMAGE_CHARACTERISTIC_UUID:
                image_char = char
                break
        
        if not image_char:
            print(f"Error: Could not find image characteristic with UUID {IMAGE_CHARACTERISTIC_UUID}")
            device.disconnect()
            return
        
        # Enable notifications for the image characteristic
        print("Enabling notifications...")
        device.writeCharacteristic(image_char.valHandle + 1, b"\x01\x00")
        
        print("Ready to receive images. Press Ctrl+C to exit.")
        print("Waiting for image data...")
        
        # Main loop to receive images
        while True:
            # Wait for notifications
            if device.waitForNotifications(5.0):
                # If we've received a complete frame, process it
                if delegate.frame_complete:
                    frame_count += 1
                    
                    # Process the image
                    try:
                        # Convert RGB565 to RGB888
                        rgb_data = rgb565_to_rgb888(delegate.image_buffer)
                        
                        # Create image from data
                        image = Image.frombytes('RGB', (WIDTH, HEIGHT), bytes(rgb_data))
                        
                        # Save image 
                        image_path = f"captured_images/image{frame_count}.jpg"
                        image.save(image_path)
                        print(f"Image saved as {image_path}")
                        
                        # Preprocess image for model
                        preprocessed_img = preprocess_image(image, INPUT_SIZE)
                        
                        # Run inference
                        print("Running inference...")
                        output = run_inference(interpreter, input_details, output_details, preprocessed_img)
                        
                        # Decode prediction
                        prediction = decode_prediction(output)
                        print(f"Prediction: {prediction}")
                        
                        # Check if first predicted class is in target classes (excluding trash)
                        if prediction[0][0] in ["cardboard", "metal", "paper", "plastic", "white-glass"]:
                            move_motor_forward()
                    
                    except Exception as e:
                        print(f"Error processing image: {e}")
                    
                    # Reset buffer for next frame
                    delegate.image_buffer = bytearray()
                    delegate.total_bytes_received = 0
                    delegate.frame_complete = False
                    
                    print("Ready for next frame...")
            else:
                print("Waiting for data...")
                
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        if 'device' in locals():
            device.disconnect()
            print("Disconnected from device")
        
        # GPIO Cleanup
        GPIO.cleanup()
        print("GPIO Clean up")

if __name__ == "__main__":
    main()
