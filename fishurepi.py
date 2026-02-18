import os
import time
import io
import struct
import select
from datetime import datetime

import serial
from PIL import Image
from picamera2 import Picamera2
from ultralytics import YOLO  # Import YOLO

import board
import neopixel
import pwmio

# ===================== SETTINGS =====================
# Timing
INTERVAL = 300                # seconds between capture cycles (5 minutes)
SERIAL_WAIT_TIMEOUT = 60      # seconds to wait for ESP32 "Done"

# Save captured JPGs locally too (optional but enabled)
SAVE_FOLDER = "images"
SAVE_LOCAL_COPY = True

# Camera
WIDTH = 480
HEIGHT = 320
JPEG_QUALITY = 60

# Serial
PORT = "/dev/ttyUSB0"
BAUD = 921600

DONE_TOKEN = "[TX3] Done"     # sender's final line
# will also accept lines ending in "Done"

# NeoPixel
NUM_PIXELS = 24
PIXEL_PIN = board.D18
BRIGHTNESS = 0.3

# Audio (PWM)
AUDIO_PIN = board.D12
# ====================================================

# ====================================================

COLORS = [
    (255, 255, 255),  # White
    (0, 255, 0),      # Green
    (0, 0, 255),      # Blue
]

FREQUENCIES = [200, 500, 1000, 2000, 5000] # Hz

# States
STATE_AUTO = "AUTO"
STATE_STATIC = "STATIC"

current_state = STATE_AUTO
static_color = (255, 255, 255) # Default static color (White)
static_freq = 1000             # Default static freq


def wait_for_done(ser: serial.Serial, timeout_s: int) -> bool:
    """Wait for ESP32 to print DONE_TOKEN (or a line ending with 'Done')."""
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
                print("[ESP32]", text)
                
                # Check for config commands from Receiver -> Sender -> Serial
                # Expect format: "CMD:MODE:STATIC", "CMD:COLOR:R,G,B", "CMD:FREQ:1000"
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
            current_state = STATE_AUTO
            print("[PI] Switched to AUTO Mode")
        elif val == "STATIC":
            current_state = STATE_STATIC
            print("[PI] Switched to STATIC Mode")
    
    elif key == "COLOR":
        # Expect R,G,B
        try:
            rgb = [int(x) for x in val.split(",")]
            if len(rgb) == 3:
                static_color = tuple(rgb)
                print(f"[PI] Static Color Set to: {static_color}")
        except ValueError:
            print("[PI] Invalid Color Format")

    elif key == "FREQ":
        try:
            freq = int(val)
            static_freq = freq
            print(f"[PI] Static Freq Set to: {static_freq}")
        except ValueError:
            print("[PI] Invalid Freq Format")

def check_serial_input(ser):
    """Check for incoming serial data without blocking (for commands)."""
    while ser.in_waiting > 0:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print("[ESP32-Async]", line)
            if line.startswith("CMD:"):
                handle_command(line[4:])


def main():
    if SAVE_LOCAL_COPY:
        os.makedirs(SAVE_FOLDER, exist_ok=True)

    # Initialize YOLO model
    print("[PI] Loading YOLO model...")
    model = YOLO("yolo26n.pt")
    
    # Serial setup
    ser = serial.Serial(PORT, BAUD, timeout=0.1) # Short timeout for non-blocking read
    time.sleep(2)

    # NeoPixel setup
    pixels = neopixel.NeoPixel(
        PIXEL_PIN,
        NUM_PIXELS,
        brightness=BRIGHTNESS,
        auto_write=False
    )
    pixels.fill((0, 0, 0))
    pixels.show()

    # Audio Logic (PWM)
    # Using 50% duty cycle for square wave
    audio_pwm = pwmio.PWMOut(AUDIO_PIN, duty_cycle=0, frequency=440, variable_frequency=True)

    def play_tone(freq):
        if freq <= 0:
            audio_pwm.duty_cycle = 0
        else:
            audio_pwm.frequency = freq
            audio_pwm.duty_cycle = 32768 # 50%

    def stop_tone():
        audio_pwm.duty_cycle = 0

    # Camera setup
    picam2 = Picamera2()
    picam2.configure(
        picam2.create_still_configuration(
            main={"size": (WIDTH, HEIGHT), "format": "RGB888"}
        )
    )
    picam2.start()
    time.sleep(1)

    print("[PI] Running... Press CTRL+C to stop.")

    step_counter = 0 # Logical step for nested cycling
    frame_counter = 0

    try:
        while True:
            # Check for incoming commands
            check_serial_input(ser)

            current_color = (0,0,0)
            current_freq = 0

            # Determine behavior based on State
            if current_state == STATE_AUTO:
                # Nested Logic:
                # Outer Cycle: Color
                # Inner Cycle: Frequency
                # We want to cycle frequencies faster or map them 1:1?
                # "cycle each frequncy to every color" -> For Color A, play Freq 1, then Freq 2...
                # So Frequency is the INNER loop (changes every step).
                # Color is the OUTER loop (changes after full freq cycle).
                
                num_freqs = len(FREQUENCIES)
                num_colors = len(COLORS)
                
                freq_idx = step_counter % num_freqs
                color_idx = (step_counter // num_freqs) % num_colors
                
                current_color = COLORS[color_idx]
                current_freq = FREQUENCIES[freq_idx]

                pixels.fill(current_color)
                pixels.show()
                play_tone(current_freq)
                
                # Wait briefly for light to stabilize and fish to react (2 seconds)
                time.sleep(2)
            else:
                # STATIC MODE
                current_color = static_color
                current_freq = static_freq
                
                pixels.fill(current_color)
                pixels.show()
                play_tone(current_freq)
                
                # Constant monitoring, smaller delay
                time.sleep(0.5)

            # 2. Capture current frame
            frame = picam2.capture_array()

            # 3. Run inference
            results = model(frame, verbose=False)
            
            # 4. Check for "fish"
            fish_detected = False
            for r in results:
                for c in r.boxes.cls:
                    class_name = model.names[int(c)]
                    if class_name.lower() == "fish":
                        fish_detected = True
                        break
                if fish_detected:
                    break

            if fish_detected:
                # In Static mode, we might want to avoid spamming if the fish sits there.
                # But per requirement "apply same logic", we'll just capture.
                # Maybe 5 min hold is ONLY for Auto? User said "automatic it will apply the same logic".
                # For Static: "user can modify color...". Implicitly, still captures?
                # Let's assume capture is desired in both, but Hold is an Auto feature?
                # Actually, "unless a fish is detected then it will stay in that color" was the Auto logic.
                # In Static, it's ALWAYS that color. So we just capture.
                
                # To prevent spam in static mode, let's impose a simple cooldown or just same 5 min wait?
                # Let's use the same send logic.

                frame_counter += 1
                print(f"\n[PI] FISH DETECTED! Frame #{frame_counter} | Color: {current_color} | Freq: {current_freq} | Mode: {current_state}")
                
                img = Image.fromarray(frame)
                buf = io.BytesIO()
                img.save(
                    buf,
                    format="JPEG",
                    quality=JPEG_QUALITY,
                    subsampling=2,
                    progressive=False
                )
                jpeg_bytes = buf.getvalue()
                jpeg_size = len(jpeg_bytes)

                if SAVE_LOCAL_COPY:
                    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                    filename = os.path.join(SAVE_FOLDER, f"{timestamp}.jpg")
                    with open(filename, "wb") as f:
                        f.write(jpeg_bytes)

                print(f"[PI] JPEG size: {jpeg_size} bytes")

                ser.write(b"\xAA\x66")
                ser.write(struct.pack("<I", jpeg_size))
                ser.write(jpeg_bytes)
                ser.flush()
                
                # We must wait for done here, blocking.
                done = wait_for_done(ser, SERIAL_WAIT_TIMEOUT)
                if done:
                    print("[PI] ✅ ESP32 finished sending")
                else:
                    print("[PI] ❌ Timeout waiting for ESP32 to finish")

                # Post-capture behavior
                if current_state == STATE_AUTO:
                    print("[PI] AUTO Mode: Holding color/freq for 5 minutes...")
                    # Ideally maintain tone? Or stop tone?
                    # "same logic with the lights" -> Lights stay ON.
                    # So Tone should stay ON? Continuous tone for 5 mins might be annoying/damaging depending on volume.
                    # But per specs, we "hold".
                    # Let's keep it playing.
                    time.sleep(300)
                    print("[PI] Resuming cycle...")
                else:
                    # Static mode: Maybe just wait a bit to avoid 1000 photos of the same fish?
                    # Let's wait 1 minute? or 5 minutes too?
                    # "if its automatic it will apply the same logic as before"
                    # User didn't specify Static capture interval. Let's start with 60s cooldown.
                    print("[PI] STATIC Mode: Cooldown 60s...")
                    time.sleep(60)

            # Move to next step (only affects Auto)
            if current_state == STATE_AUTO:
                step_counter += 1
            else:
                # In static mode, we don't advance the step counter
                # but we still need to ensure the tone is stopped if no fish is detected
                # and then re-played on the next iteration.
                pass # No change to step_counter in static mode

    except KeyboardInterrupt:
        print("\n[PI] Stopping...")

    finally:
        try:
            pixels.fill((0, 0, 0))
            pixels.show()
        except Exception:
            pass
            
        try:
            # Stop Audio
            audio_pwm.deinit()
        except Exception:
            pass

        try:
            ser.close()
        except Exception:
            pass

        try:
            picam2.stop()
        except Exception:
            pass

        print("[PI] Finished")


if __name__ == "__main__":
    main()