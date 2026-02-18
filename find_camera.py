import cv2
import os

def scan_cameras():
    print("--- Camera Diagnostic Script ---")
    
    # 1. Check if hardware is detected by system
    print("\nChecking system devices...")
    video_devices = [f for f in os.listdir('/dev') if f.startswith('video')]
    print(f"Found devices: {video_devices}")

    # 2. Test specific Raspberry Pi Pipelines
    print("\nTesting Raspberry Pi specific pipelines...")
    pipelines = [
        ("GStreamer Simple", "libcamerasrc ! videoconvert ! appsink"),
        ("GStreamer Detailed", "libcamerasrc ! video/x-raw,width=640,height=480 ! videoconvert ! appsink"),
        ("V4L2 Index 0", 0, cv2.CAP_V4L2),
        ("V4L2 Index 1", 1, cv2.CAP_V4L2),
        ("Default Index 0", 0),
        ("Default Index 1", 1),
    ]

    for name, *args in pipelines:
        print(f"\nTesting {name}...")
        try:
            if len(args) == 2:
                cap = cv2.VideoCapture(args[0], args[1])
            else:
                cap = cv2.VideoCapture(args[0])
            
            if not cap.isOpened():
                print(f"  [FAIL] Could not open {name}")
                continue
                
            ret, frame = cap.read()
            if ret:
                print(f"  [SUCCESS] Successfully captured frame from {name}!")
                cap.release()
                return args[0], (args[1] if len(args) == 2 else None)
            else:
                print(f"  [FAIL] Opened, but could not read frame from {name}")
            cap.release()
        except Exception as e:
            print(f"  [ERROR] {e}")

    print("\n--- Diagnostic Finished ---")
    return None, None

if __name__ == "__main__":
    scan_cameras()
