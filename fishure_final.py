import os
import time
import io
import struct
import subprocess
from datetime import datetime

import serial
import cv2
import numpy as np
from PIL import Image
from ultralytics import YOLO

import board
import neopixel
import pwmio

# --- CONFIGURATION ---
WIDTH, HEIGHT = 640, 480
PIXEL_PIN = board.D18
NUM_PIXELS = 10
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 921600
YOLO_MODEL_PATH = "yolo26n.pt"

# --- HARDWARE SETUP ---
pixels = neopixel.NeoPixel(PIXEL_PIN, NUM_PIXELS, brightness=0.3, auto_write=False)
audio_pwm = pwmio.PWMOut(board.D12, duty_cycle=0, frequency=440, variable_frequency=True)

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"[PI] Serial connected to {SERIAL_PORT}")
except Exception as e:
    print(f"[PI] Serial Error: {e}")
    ser = None

print("[PI] Loading YOLO model...")
model = YOLO(YOLO_MODEL_PATH)

def capture_pi_cam():
    try:
        cmd = ["rpicam-still", "-t", "5", "-n", "--stdout", "--width", str(WIDTH), "--height", str(HEIGHT), "-e", "jpg", "--immediate"]
        result = subprocess.run(cmd, capture_output=True, check=True)
        nparr = np.frombuffer(result.stdout, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if frame is not None:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return None
    except Exception as e:
        print(f"[PI] Capture Error: {e}")
        return None

def main():
    print("[PI] Starting loop (Subprocess Camera Mode)...")
    last_send_time = 0
    
    try:
        while True:
            # 1. Listen for commands from Serial (optional)
            if ser and ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"[ESP32] {line}")

            # 2. Capture Frame
            frame = capture_pi_cam()
            if frame is None:
                time.sleep(0.5)
                continue

            # 3. Run Inference
            results = model(frame, verbose=False)
            fish_count = 0
            for r in results:
                for c in r.boxes.cls:
                    if model.names[int(c)].lower() == "fish":
                        fish_count += 1

            print(f"[PI] Detected {fish_count} fish")

            # 4. Update NeoPixels
            if fish_count > 0:
                pixels.fill((0, 255, 0)) # Green
            else:
                pixels.fill((255, 0, 0)) # Red
            pixels.show()

            # 5. Send data back to Serial
            # Limit image sending frequency to avoid clogging (max 1 frame per 2 seconds)
            if ser and (time.time() - last_send_time > 2):
                try:
                    # Convert to JPEG for Serial Transmission
                    img = Image.fromarray(frame)
                    buf = io.BytesIO()
                    img.save(buf, format="JPEG", quality=60)
                    jpeg_bytes = buf.getvalue()
                    jpeg_size = len(jpeg_bytes)
                    
                    ser.write(b"\xAA\x66") # Header
                    ser.write(struct.pack("<I", jpeg_size)) # Size
                    ser.write(jpeg_bytes) # Payload
                    ser.flush()
                    
                    last_send_time = time.time()
                    print(f"[PI] Sent image feedback ({jpeg_size} bytes)")
                except Exception as e:
                    print(f"[PI] Serial Send Error: {e}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[PI] Stopping...")
    finally:
        pixels.fill((0, 0, 0))
        pixels.show()
        if ser:
            ser.close()
        print("[PI] Finished")

if __name__ == "__main__":
    main()
