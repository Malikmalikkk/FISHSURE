import time
import board
import neopixel
import pwmio

print("--- Hardware Diagnostic ---")

# 1. Test NeoPixels
try:
    print("Testing NeoPixels on D18...")
    pixels = neopixel.NeoPixel(board.D18, 10, brightness=0.2, auto_write=False)
    pixels.fill((0, 255, 0))
    pixels.show()
    print("  [SUCCESS] NeoPixels initialized and set to GREEN")
    time.sleep(1)
    pixels.fill((0, 0, 0))
    pixels.show()
except Exception as e:
    print(f"  [FAIL] NeoPixel Error: {e}")

# 2. Test PWM
try:
    print("\nTesting PWM on D12...")
    audio_pwm = pwmio.PWMOut(board.D12, duty_cycle=0, frequency=440, variable_frequency=True)
    audio_pwm.frequency = 880
    audio_pwm.duty_cycle = 32768
    print("  [SUCCESS] PWM initialized. Sound should be playing.")
    time.sleep(1)
    audio_pwm.duty_cycle = 0
    audio_pwm.deinit()
except Exception as e:
    print(f"  [FAIL] PWM Error: {e}")

print("\n--- Diagnostic Finished ---")
