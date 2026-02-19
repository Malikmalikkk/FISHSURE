import os
import time
import io
import struct
import pickle
import subprocess
from datetime import datetime

import serial
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO
from skopt import Optimizer
from skopt.space import Integer
from skopt.utils import use_named_args

# --- V3 WIDE & COMPATIBILITY CHECKS ---
if np.version.version.startswith('2.'):
    print("[WARNING] NumPy 2.x detected. If you hit 'Failed to initialize NumPy' error, please run: pip install 'numpy<2.0'")

import board
import neopixel
import pwmio

# --- CONFIGURATION ---
WIDTH, HEIGHT = 640, 480
PIXEL_PIN = board.D18
NUM_PIXELS = 24
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 921600
YOLO_MODEL_PATH = "best.pt"
OPTIMIZER_SAVE_PATH = "optimizer.pkl"
CONF_THRESHOLD = 0.9  # Model confidence threshold (0.0 to 1.0)
SAVE_FOLDER = "captures" # Folder to save detected images

# Bayesian Opt Settings
N_INITIAL_POINTS = 10
SEARCH_SPACE = [
    Integer(0, 255, name='red'),
    Integer(0, 255, name='green'),
    Integer(0, 255, name='blue'),
    Integer(500, 5000, name='freq')
]

# --- MODE & STATIC GLOBALS ---
current_mode = "AUTO"  # "AUTO" or "STATIC"
static_r, static_g, static_b = 255, 255, 255
static_freq = 1000

# --- HARDWARE SETUP ---
pixels = neopixel.NeoPixel(PIXEL_PIN, NUM_PIXELS, brightness=1.0, auto_write=False)
audio_pwm = pwmio.PWMOut(board.D12, duty_cycle=0, frequency=440, variable_frequency=True)

ser = None
try:
    if os.path.exists(SERIAL_PORT):
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print(f"[PI] Connected to Serial port: {SERIAL_PORT}")
    else:
        print(f"[PI] Serial port {SERIAL_PORT} not found. Running in simulated serial mode.")
except Exception as e:
    print(f"[PI] Serial Error: {e}")

print("[PI] Loading YOLO model...")
model = YOLO(YOLO_MODEL_PATH)

# --- OPTIMIZER SETUP ---
dimensions = SEARCH_SPACE
if os.path.exists(OPTIMIZER_SAVE_PATH):
    try:
        with open(OPTIMIZER_SAVE_PATH, "rb") as f:
            opt = pickle.load(f)
    except:
        opt = Optimizer(dimensions=dimensions, n_initial_points=N_INITIAL_POINTS, acq_func="gp_hedge")
else:
    opt = Optimizer(dimensions=dimensions, n_initial_points=N_INITIAL_POINTS, acq_func="gp_hedge")

def capture_pi_cam():
    try:
        # -t 500 allows autofocus on Pi Cam v3 to settle
        cmd = ["rpicam-still", "-t", "500", "-n", "-o", "-", "--width", str(WIDTH), "--height", str(HEIGHT), "-e", "jpg"]
        result = subprocess.run(cmd, capture_output=True, check=True)
        
        if not result.stdout:
            print(f"[PI] Camera Error: stdout is empty. Stderr: {result.stderr.decode()}")
            return None
            
        print(f"[PI] Captured {len(result.stdout)} bytes from camera")
        nparr = np.frombuffer(result.stdout, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if frame is not None:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return None
    except subprocess.CalledProcessError as e:
        print(f"[PI] Camera Command Failed (status {e.returncode})")
        if e.stderr:
            print(f"[PI] Camera Stderr: {e.stderr.decode()}")
        return None
    except Exception as e:
        print(f"[PI] Camera Exception: {e}")
        return None

@use_named_args(dimensions)
def objective(red, green, blue, freq):
    global trial_counter
    # Apply settings
    pixels.fill((red, green, blue))
    pixels.show()
    audio_pwm.frequency = int(freq)
    audio_pwm.duty_cycle = 32768
    
    time.sleep(2) # Wait for fish to react
    
    # Capture & Count
    frame = capture_pi_cam()
    fish_count = 0
    jpeg_bytes = None
    if frame is not None:
        # Run inference with confidence threshold
        results = model(frame, verbose=False, conf=CONF_THRESHOLD)
        for r in results:
            for c in r.boxes.cls:
                # Use name check for safety
                if model.names[int(c)].lower() == "fish":
                    fish_count += 1
        
        # Get annotated frame with bounding boxes
        annotated_frame = results[0].plot()  # Returns BGR
        annotated_frame_rgb = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
        
        # Convert to PIL for custom text overlay
        img = Image.fromarray(annotated_frame_rgb)
        draw = ImageDraw.Draw(img)
        info_text = f"Trial: {trial_counter}\nFish: {fish_count}\nRGB: {red},{green},{blue}\nFreq: {freq}Hz"
        draw.text((10, 10), info_text, fill=(255, 0, 0))
        
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=60)
        jpeg_bytes = buf.getvalue()

        # Save local copy ONLY if fish are detected
        if fish_count > 0:
            os.makedirs(SAVE_FOLDER, exist_ok=True)
            filename = f"trial_{trial_counter}_fish_{fish_count}.jpg"
            with open(os.path.join(SAVE_FOLDER, filename), "wb") as f:
                f.write(jpeg_bytes)
    
        status_msg = "Saved locally" if fish_count > 0 else "Skipped local save"
        print(f"[BAYES] Trial {trial_counter}: RGB=({red},{green},{blue}) Freq={freq}Hz -> Fish={fish_count} ({status_msg})")

    # Send to Serial (matching fishurepi.py protocol)
    if ser and jpeg_bytes:
        try:
            jpeg_size = len(jpeg_bytes)
            ser.write(b"\xAA\x66") # Header
            ser.write(struct.pack("<I", jpeg_size)) # Size
            ser.write(jpeg_bytes) # Payload
            ser.flush()
            print(f"[PI] Sent image ({jpeg_size} bytes)")
        except Exception as e:
            print(f"[PI] Serial Send Error: {e}")

    trial_counter += 1
    return -fish_count # Negate because skopt minimizes

def handle_serial_commands():
    global current_mode, static_r, static_g, static_b, static_freq
    if ser and ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("CMD:"):
                print(f"[PI] Received Command: {line}")
                parts = line.split(":")
                cmd_type = parts[1]
                
                if cmd_type == "MODE":
                    current_mode = parts[2]
                    print(f"[PI] Switching to {current_mode} mode")
                elif cmd_type == "COLOR":
                    vals = parts[2].split(",")
                    static_r, static_g, static_b = int(vals[0]), int(vals[1]), int(vals[2])
                    print(f"[PI] Static Color updated: {static_r},{static_g},{static_b}")
                elif cmd_type == "FREQ":
                    static_freq = int(parts[2])
                    print(f"[PI] Static Freq updated: {static_freq}Hz")
        except Exception as e:
            print(f"[PI] Serial Parse Error: {e}")

trial_counter = 0

def main():
    print("[PI] Starting Bayes Opt + Serial Listener...")
    try:
        while True:
            # 1. Check for commands from ESP32
            handle_serial_commands()

            if current_mode == "AUTO":
                # 2. EVALUATE NEXT BAYES STEP
                next_x = opt.ask()
                
                # evaluate (handles hardware + detection + serial tx)
                f_val = objective(next_x) 
                
                opt.tell(next_x, f_val)
                
                # Save state
                with open(OPTIMIZER_SAVE_PATH, "wb") as f:
                    pickle.dump(opt, f)
            else:
                # 3. STATIC MODE: Just apply values and capture
                # We must pass a list [r, g, b, f] because of the @use_named_args decorator
                objective([static_r, static_g, static_b, static_freq])
                time.sleep(1) # Pacing in static mode

    except KeyboardInterrupt:
        print("\n[PI] Stopping...")
    finally:
        pixels.fill((0, 0, 0))
        pixels.show()
        audio_pwm.deinit()
        if ser: ser.close()

if __name__ == "__main__":
    main()
