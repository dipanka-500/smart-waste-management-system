from bluepy import btle
import time
import struct
import numpy as np
from PIL import Image
import os
import tensorflow as tf  # You can replace with another ML framework if needed

# BLE device information
mac_address = "17:ef:77:33:82:44"  
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef1"  # Service UUID from Arduino code
IMAGE_CHARACTERISTIC_UUID = "12345678-1234-5678-1234-56789abcdef2"  # Image characteristic UUID
RESULT_CHARACTERISTIC_UUID = "12345678-1234-5678-1234-56789abcdef3"  # New UUID for sending results back

# Image parameters (must match camera settings on Arduino)
WIDTH = 176  # QCIF width
HEIGHT = 144  # QCIF height
BYTES_PER_PIXEL = 2  # RGB565 format
BYTES_PER_FRAME = WIDTH * HEIGHT * BYTES_PER_PIXEL

# Model settings
MODEL_PATH = "model.h"  # path to model
INPUT_SIZE = (224, 224)  # Model input size, adjust as needed
NUM_CLASSES = 10  # Number of classes in your model

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

def load_tflite_model(model_path):
    """Load TensorFlow Lite model"""
    try:
        interpreter = tf.lite.Interpreter(model_path=model_path)
        interpreter.allocate_tensors()
        
        # Get input and output details
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        
        print(f"Model loaded. Input shape: {input_details[0]['shape']}, Output shape: {output_details[0]['shape']}")
        
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
    # Set input tensor
    interpreter.set_tensor(input_details[0]['index'], image)
    
    # Run inference
    interpreter.invoke()
    
    # Get output tensor
    output_data = interpreter.get_tensor(output_details[0]['index'])
    
    return output_data

def decode_prediction(prediction, top=1):
    """Convert model output to class prediction"""
    # For simplicity, assuming prediction is a probabilities array
    # You might need to modify this based on your model's output format
    top_indices = np.argsort(prediction[0])[-top:][::-1]
    results = [
        (idx, float(prediction[0][idx]))
        for idx in top_indices
    ]
    return results

def send_prediction_to_arduino(device, result_char, prediction):
    """Send prediction result back to Arduino"""
    try:
        # For simplicity, we'll send the class index and confidence as bytes
        class_idx, confidence = prediction[0]
        
        # Pack as a simple struct: class index (1 byte) and confidence (4 bytes float)
        result_bytes = struct.pack('<Bf', class_idx, confidence)
        
        # Send data in chunks if needed (BLE has packet size limitations)
        max_chunk = 20  # Safe BLE packet size
        for i in range(0, len(result_bytes), max_chunk):
            chunk = result_bytes[i:i+max_chunk]
            result_char.write(chunk)
            time.sleep(0.01)  # Small delay between chunks
            
        print(f"Sent prediction: Class {class_idx} with confidence {confidence:.4f}")
        return True
    except Exception as e:
        print(f"Error sending prediction: {e}")
        return False

def main():
    frame_count = 0
    
    # Create output directory if it doesn't exist
    os.makedirs("captured_images", exist_ok=True)
    
    # Load ML model
    print("Loading model...")
    interpreter, input_details, output_details = load_tflite_model(MODEL_PATH)
    
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
        
        # Find the image and result characteristics
        image_char = None
        result_char = None
        
        for char in characteristics:
            if char.uuid == IMAGE_CHARACTERISTIC_UUID:
                image_char = char
            elif char.uuid == RESULT_CHARACTERISTIC_UUID:
                result_char = char
        
        if not image_char:
            print(f"Error: Could not find image characteristic with UUID {IMAGE_CHARACTERISTIC_UUID}")
            device.disconnect()
            return
            
        if not result_char:
            print(f"Warning: Could not find result characteristic with UUID {RESULT_CHARACTERISTIC_UUID}")
            print("Will not be able to send predictions back to Arduino")
        
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
                        
                        # Save image (optional)
                        image_path = f"captured_images/captured_image_{frame_count}.jpg"
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
                        
                        # Send prediction back to Arduino
                        if result_char:
                            send_prediction_to_arduino(device, result_char, prediction)
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

if __name__ == "__main__":
    main()
