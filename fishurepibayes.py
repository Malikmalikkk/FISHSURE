import os
import time
import io
import struct
import select
import pickle
from datetime import datetime

import serial
from PIL import Image, ImageDraw, ImageFont
from picamera2 import Picamera2
from ultralytics import YOLO

import board
import neopixel
import pwmio

# Bayesian Optimization Imports
from skopt import Optimizer
from skopt.space import Integer
from skopt.utils import use_named_args

# ===================== SETTINGS =====================
# Persistence
OPTIMIZER_FILE = "optimizer.pkl"

# Optimization Settings
TRIAL_DURATION = 15           # Seconds to wait after applying settings before measuring
N_INITIAL_POINTS = 10         # Random points before model kicks in

# Serial
PORT = "/dev/ttyUSB0"
BAUD = 921600
SERIAL_WAIT_TIMEOUT = 60
DONE_TOKEN = "[TX3] Done"

# Hardware
PIXEL_PIN = board.D18
NUM_PIXELS = 24
BRIGHTNESS = 0.3
AUDIO_PIN = board.D12

# Camera
WIDTH = 480
HEIGHT = 320
JPEG_QUALITY = 60
SAVE_FOLDER = "images_bayes"
SAVE_LOCAL_COPY = True
# ====================================================

COLORS = [
    (255, 255, 255),  # White (Index 0)
    (0, 255, 0),      # Green (Index 1)
    (0, 0, 255),      # Blue  (Index 2)
]

# States
STATE_AUTO = "AUTO"   # Bayesian Optimization Mode
STATE_STATIC = "STATIC" # Manual Control Mode

current_state = STATE_AUTO
static_color = (255, 255, 255)
static_freq = 1000

# Bayesian Optimizer Setup
# Space: Color Index (0-2), Frequency (200-5000)
dimensions = [
    Integer(0, 2, name='color_idx'),
    Integer(200, 5000, name='frequency')
]

# Load or Create Optimizer
if os.path.exists(OPTIMIZER_FILE):
    print(f"[BAYES] Loading existing optimizer from {OPTIMIZER_FILE}")
    try:
        with open(OPTIMIZER_FILE, "rb") as f:
            opt = pickle.load(f)
    except Exception as e:
        print(f"[BAYES] Failed to load optimizer: {e}. Creating new.")
        opt = Optimizer(dimensions=dimensions, n_initial_points=N_INITIAL_POINTS,  acq_func="GP_Hedge")
else:
    print("[BAYES] Creating new optimizer")
    opt = Optimizer(dimensions=dimensions, n_initial_points=N_INITIAL_POINTS, acq_func="GP_Hedge")


def wait_for_done(ser: serial.Serial, timeout_s: int) -> bool:
    """Wait for ESP32 with command processing."""
    deadline = time.time() + timeout_s
    rxbuf = b""

    while time.time() < deadline:
        r, _, _ = select.select([ser], [], [], 0.1)
        if not r:
            continue

        data = ser.read(1024)
        if not data:
            continue

        rxbuf += data
        while b"\n" in rxbuf:
            line, rxbuf = rxbuf.split(b"\n", 1)
            text = line.decode(errors="ignore").strip()
            if text:
                # Check for config commands
                if text.startswith("CMD:"):
                    handle_command(text[4:])

            if (DONE_TOKEN in text) or text.endswith("Done"):
                return True
    return False

def handle_command(cmd_str):
    global current_state, static_color, static_freq
    cmd_str = cmd_str.strip().upper()
    print(f"[PI] Received Command: {cmd_str}")
    
    parts = cmd_str.split(":")
    if len(parts) < 2:
        return

    key = parts[0]
    val = parts[1]

    if key == "MODE":
        if val == "AUTO":
            if current_state != STATE_AUTO:
                print("[PI] Switched to AUTO (Bayesian Mode)")
            current_state = STATE_AUTO
        elif val == "STATIC":
            if current_state != STATE_STATIC:
                print("[PI] Switched to STATIC (Manual Mode)")
            current_state = STATE_STATIC
    
    elif key == "COLOR":
        try:
            rgb = [int(x) for x in val.split(",")]
            if len(rgb) == 3:
                static_color = tuple(rgb)
                print(f"[PI] Static Color Set to: {static_color}")
        except ValueError: pass

    elif key == "FREQ":
        try:
            freq = int(val)
            static_freq = freq
            print(f"[PI] Static Freq Set to: {static_freq}")
        except ValueError: pass

def check_serial_input(ser):
    while ser.in_waiting > 0:
        line = ser.readline().decode(errors="ignore").strip()
        if line and line.startswith("CMD:"):
            handle_command(line[4:])

def main():
    if SAVE_LOCAL_COPY:
        os.makedirs(SAVE_FOLDER, exist_ok=True)

    print("[PI] Loading YOLO model...")
    model = YOLO("yolo26n.pt")
    
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(2)

    pixels = neopixel.NeoPixel(PIXEL_PIN, NUM_PIXELS, brightness=BRIGHTNESS, auto_write=False)
    pixels.fill((0, 0, 0)); pixels.show()

    audio_pwm = pwmio.PWMOut(AUDIO_PIN, duty_cycle=0, frequency=440, variable_frequency=True)

    def play_tone(freq):
        if freq <= 0: audio_pwm.duty_cycle = 0
        else:
            audio_pwm.frequency = int(freq)
            audio_pwm.duty_cycle = 32768

    picam2 = Picamera2()
    picam2.configure(picam2.create_still_configuration(main={"size": (WIDTH, HEIGHT), "format": "RGB888"}))
    picam2.start()
    time.sleep(1)

    print("[PI] Running Bayes Opt... Ctrl+C to stop.")

    trial_num = 0

    try:
        while True:
            check_serial_input(ser)

            # ================= AUTO (BAYESIAN) MODE =================
            if current_state == STATE_AUTO:
                # 1. Ask Optimizer for next point
                suggested = opt.ask()
                color_idx = suggested[0]
                freq = suggested[1]
                
                # Apply Settings
                color = COLORS[color_idx]
                pixels.fill(color)
                pixels.show()
                play_tone(freq)
                
                print(f"\n[BAYES] Trial {trial_num}: Testing Color={color} (Idx {color_idx}), Freq={freq}Hz")
                
                # 2. WAIT for fish to respond (Trial Duration)
                # We can't block strictly for 15s if we want to remain responsive to commands?
                # But for optimization consistency, we usually commit to a trial.
                # Let's break it into chunks to check serial
                for _ in range(TRIAL_DURATION):
                    time.sleep(1)
                    check_serial_input(ser)
                    if current_state != STATE_AUTO: break # Abort if mode changed
                
                if current_state != STATE_AUTO: continue # Skip measurement if mode changed

                # 3. MEASURE (Capture & Count)
                frame = picam2.capture_array()
                results = model(frame, verbose=False)
                
                fish_count = 0
                for r in results:
                    for c in r.boxes.cls:
                        if model.names[int(c)].lower() == "fish":
                            fish_count += 1
                
                print(f"[BAYES] Result: Discovered {fish_count} fish.")

                # 4. TELL Optimizer (Minimize negative count -> Maximize count)
                # skopt minimizes, so we pass -fish_count
                opt.tell(suggested, -fish_count)
                
                # Save Optimizer state
                with open(OPTIMIZER_FILE, "wb") as f:
                    pickle.dump(opt, f)

                # 5. Send Feedback Image to User
                # Draw stats on image
                img = Image.fromarray(frame)
                draw = ImageDraw.Draw(img)
                # Simple text overlay
                info_text = f"Trial {trial_num}\nFreq: {freq}Hz\nColor: {color_idx}\nFish: {fish_count}"
                draw.text((10, 10), info_text, fill=(255, 0, 0)) # Red text

                buf = io.BytesIO()
                img.save(buf, format="JPEG", quality=JPEG_QUALITY, subsampling=2)
                jpeg_bytes = buf.getvalue()
                jpeg_size = len(jpeg_bytes)

                if SAVE_LOCAL_COPY:
                    ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                    with open(os.path.join(SAVE_FOLDER, f"{ts}_trial{trial_num}_fish{fish_count}.jpg"), "wb") as f:
                        f.write(jpeg_bytes)

                ser.write(b"\xAA\x66")
                ser.write(struct.pack("<I", jpeg_size))
                ser.write(jpeg_bytes)
                ser.flush()
                wait_for_done(ser, SERIAL_WAIT_TIMEOUT)

                trial_num += 1

            # ================= STATIC (MANUAL) MODE =================
            else:
                pixels.fill(static_color)
                pixels.show()
                play_tone(static_freq)
                
                # Capture freq faster (0.5s) for responsive monitoring
                time.sleep(0.5)
                
                frame = picam2.capture_array()
                results = model(frame, verbose=False)
                
                # Just detect existence for manual mode standard behavior?
                # Or count? Let's count/detect.
                fish_detected = False
                for r in results:
                    for c in r.boxes.cls:
                        if model.names[int(c)].lower() == "fish":
                            fish_detected = True; break
                
                if fish_detected:
                     print(f"[STATIC] Fish Detected! Color={static_color}, Freq={static_freq}")
                     img = Image.fromarray(frame)
                     buf = io.BytesIO()
                     img.save(buf, format="JPEG", quality=JPEG_QUALITY, subsampling=2)
                     jpeg_bytes = buf.getvalue()
                     jpeg_size = len(jpeg_bytes)
                     
                     ser.write(b"\xAA\x66")
                     ser.write(struct.pack("<I", jpeg_size))
                     ser.write(jpeg_bytes)
                     ser.flush()
                     wait_for_done(ser, SERIAL_WAIT_TIMEOUT)
                     
                     # Cooldown?
                     time.sleep(1) 

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        pixels.fill((0,0,0)); pixels.show()
        audio_pwm.deinit()
        ser.close()
        picam2.stop()
