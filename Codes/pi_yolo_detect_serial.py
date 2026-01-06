#!/usr/bin/env python3
"""
pi_yolo_detect_serial.py

Live detection on Raspberry Pi (Picamera2 or USB camera) with Ultralytics YOLO.
Sends 'metal\n' / 'glass\n' / 'paper\n' to Arduino via serial after debounce and waits for ACK.

Usage example:
  python3 pi_yolo_detect_serial.py --model runs/detect/train/weights/best.pt --source picamera0 --resolution 1280x720 --port /dev/ttyACM0

"""

import argparse
import time
import sys
import os

import cv2
import numpy as np
import serial

# Try to import Picamera2 if user wants picamera
USE_PICAMERA2 = False
try:
    from picamera2 import Picamera2
    USE_PICAMERA2 = True
except Exception:
    # not installed or not available; OK for USB cameras
    USE_PICAMERA2 = False

# ultralytics
try:
    from ultralytics import YOLO
except Exception as e:
    print("ERROR: ultralytics not available. Install with: pip3 install ultralytics")
    print("Exception:", e)
    sys.exit(1)


# ----------------------------
# CLI arguments
# ----------------------------
parser = argparse.ArgumentParser()
parser.add_argument('--model', required=True, help='Path to Ultralytics .pt model (e.g. runs/detect/train/weights/best.pt)')
parser.add_argument('--source', required=True, help='Source: usb0, usb1, picamera0, video.mp4, or image file/folder')
parser.add_argument('--resolution', default=None, help='Resolution WIDTHxHEIGHT (e.g. 1280x720). If omitted uses camera default.')
parser.add_argument('--thresh', type=float, default=0.60, help='Confidence threshold (default 0.60)')
parser.add_argument('--consec', type=int, default=3, help='Consecutive frames required (default 3)')
parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port to Arduino (default /dev/ttyACM0). Omit to run without Arduino.')
parser.add_argument('--baud', type=int, default=9600, help='Serial baud rate (default 9600)')
parser.add_argument('--ack-timeout', type=float, default=40.0, help='Seconds to wait for Arduino ACK (default 40s)')
parser.add_argument('--cooldown', type=float, default=2.0, help='Cooldown seconds between sends (default 2s)')
parser.add_argument('--show', action='store_true', help='Show OpenCV window (set if you have desktop or X forwarding).')
args = parser.parse_args()

MODEL_PATH = args.model
SOURCE = args.source
RESOLUTION = args.resolution
CONF_THRESH = args.thresh
CONSECUTIVE_FRAMES = args.consec
SERIAL_PORT = args.port if args.port.lower() != "none" else None
BAUDRATE = args.baud
ACK_TIMEOUT_SEC = args.ack_timeout
COOLDOWN_SEC = args.cooldown
SHOW_WINDOW = args.show

TARGET_CLASSES = {"metal", "glass", "paper"}  # ensure these match names in your model

ACK_KEYWORDS = ["cycle complete", "back to hold", "done", "ok"]  # keywords from Arduino printing

# ----------------------------
# Validate model file
# ----------------------------
if not os.path.exists(MODEL_PATH):
    print("ERROR: Model file not found:", MODEL_PATH)
    sys.exit(1)

# ----------------------------
# Load YOLO model
# ----------------------------
print("Loading YOLO model:", MODEL_PATH)
model = YOLO(MODEL_PATH, task='detect')

# get labels mapping
try:
    labels = model.names
except Exception:
    labels = {}

# ----------------------------
# Open serial
# ----------------------------
ser = None
if SERIAL_PORT:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        time.sleep(2.0)  # allow Arduino reset if any
        print(f"Opened serial port {SERIAL_PORT} @ {BAUDRATE}")
    except Exception as e:
        print("Warning: could not open serial port:", e)
        ser = None
else:
    print("Serial disabled (no port). Script will run but not actually send commands.")

def send_serial_line(msg: str):
    if not ser:
        print("[SIM] ->", msg.strip())
        return False
    try:
        ser.write(msg.encode())
        return True
    except Exception as e:
        print("Serial write error:", e)
        return False

def read_serial_line_nonblocking():
    if not ser:
        return None
    try:
        line = ser.readline()
        if not line:
            return None
        try:
            return line.decode(errors='replace').strip()
        except:
            return repr(line)
    except Exception as e:
        print("Serial read error:", e)
        return None

# ----------------------------
# Initialize camera/source
# ----------------------------
use_picamera = False
cap = None
picam = None

if SOURCE.startswith("usb"):
    # expect usb0, usb1 ...
    try:
        cam_idx = int(SOURCE[3:])
    except:
        print("Invalid usb source. Use usb0, usb1 etc.")
        sys.exit(1)
    cap = cv2.VideoCapture(cam_idx)
    if RESOLUTION:
        w, h = RESOLUTION.split('x')
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(w))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(h))
elif SOURCE.startswith("picamera"):
    if not USE_PICAMERA2:
        print("Picamera2 not available. Install picamera2 package or use usb camera.")
        sys.exit(1)
    use_picamera = True
    picam = Picamera2()
    if RESOLUTION:
        w, h = map(int, RESOLUTION.split('x'))
    else:
        # default resolution
        w, h = 1280, 720
    config = picam.create_preview_configuration(main={"format": 'XRGB8888', "size": (w, h)})
    picam.configure(config)
    picam.start()
elif os.path.isfile(SOURCE):
    # video or image file
    _, ext = os.path.splitext(SOURCE)
    ext = ext.lower()
    if ext in ['.jpg', '.jpeg', '.png', '.bmp']:
        cap = None
        image_mode = True
        frame_image = cv2.imread(SOURCE)
    else:
        cap = cv2.VideoCapture(SOURCE)
        if RESOLUTION:
            w, h = RESOLUTION.split('x')
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(w))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(h))
else:
    print("Unsupported source:", SOURCE)
    sys.exit(1)

# ----------------------------
# Colors for boxes
# ----------------------------
bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106),
               (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

# ----------------------------
# Main loop: detection, send, wait for ACK
# ----------------------------
consecutive = 0
last_label = None
last_sent_time = 0.0
waiting_for_ack = False
ack_start_time = 0.0

print("Starting capture. Press Ctrl+C to quit or close window if shown.")

try:
    while True:
        # poll serial to print any incoming lines (so they are visible while waiting)
        sline = read_serial_line_nonblocking()
        if sline:
            print("[Arduino] ", sline)
            if waiting_for_ack:
                low = sline.lower()
                for kw in ACK_KEYWORDS:
                    if kw in low:
                        waiting_for_ack = False
                        print("ACK detected from Arduino (keyword):", kw)
                        consecutive = 0
                        last_label = None
                        break

        # capture frame
        if use_picamera:
            frame_bgra = picam.capture_array()
            frame = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)
        elif cap is not None:
            ret, frame = cap.read()
            if not ret:
                print("Frame not read. Exiting.")
                break
        else:
            # image file mode
            frame = frame_image.copy()

        # optionally resize (already set earlier)
        # run detection only if not waiting for ack
        if not waiting_for_ack:
            # model expects BGR numpy frames directly
            results = model(frame, verbose=False)
            detections = results[0].boxes

            # draw boxes and find best target
            chosen = None
            best_conf = 0.0
            object_count = 0

            for i in range(len(detections)):
                xyxy_tensor = detections[i].xyxy.cpu()
                xyxy = xyxy_tensor.numpy().squeeze()
                xmin, ymin, xmax, ymax = xyxy.astype(int)

                classidx = int(detections[i].cls.item())
                classname = labels.get(classidx, str(classidx)) if isinstance(labels, dict) else labels[classidx]
                conf = float(detections[i].conf.item())

                if conf >= CONF_THRESH:
                    color = bbox_colors[classidx % len(bbox_colors)]
                    cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                    label_text = f"{classname}: {conf:.2f}"
                    cv2.putText(frame, label_text, (xmin, max(ymin, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)
                    object_count += 1

                    lab = classname.lower().strip()
                    if lab in TARGET_CLASSES and conf > best_conf:
                        best_conf = conf
                        chosen = (lab, conf, (xmin, ymin, xmax, ymax))

            # show overlay information
            cv2.putText(frame, f"Objects: {object_count}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

            # debounce / confirmation
            if chosen:
                lab, conf, _ = chosen
                if lab == last_label:
                    consecutive += 1
                else:
                    last_label = lab
                    consecutive = 1

                cv2.putText(frame, f"Candidate: {lab} ({consecutive}/{CONSECUTIVE_FRAMES}) conf={conf:.2f}", (10,60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,200), 2)

                if consecutive >= CONSECUTIVE_FRAMES:
                    now = time.time()
                    if now - last_sent_time >= COOLDOWN_SEC:
                        msg = lab + "\n"
                        ok = send_serial_line(msg)
                        if ok:
                            print("Sent to Arduino:", lab)
                        else:
                            print("Simulated send (no serial):", lab)
                        last_sent_time = now
                        waiting_for_ack = True
                        ack_start_time = time.time()
                        consecutive = 0
                        last_label = None
                        cv2.putText(frame, f"SENT: {lab}", (10,100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 3)
                    else:
                        cd_rem = COOLDOWN_SEC - (now - last_sent_time)
                        cv2.putText(frame, f"Cooldown: {cd_rem:.1f}s", (10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,200), 2)
            else:
                consecutive = 0
                last_label = None
        else:
            # waiting for ack - overlay and timeout
            elapsed = time.time() - ack_start_time
            cv2.putText(frame, f"Waiting for Arduino ACK... {int(elapsed)}s", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,200,255), 2)
            if elapsed > ACK_TIMEOUT_SEC:
                print(f"ACK timeout after {ACK_TIMEOUT_SEC}s - resuming detection.")
                waiting_for_ack = False
                consecutive = 0
                last_label = None

        # show window if requested
        if SHOW_WINDOW:
            cv2.imshow("YOLO Live (press q to quit)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("User requested exit.")
                break
        else:
            # If no window, allow short sleep to avoid 100% CPU loop
            time.sleep(0.01)

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    print("Cleaning up...")
    try:
        if use_picamera and picam:
            picam.stop()
        if cap is not None:
            cap.release()
    except:
        pass
    if ser:
        try:
            ser.close()
        except:
            pass
    if SHOW_WINDOW:
        cv2.destroyAllWindows()
    print("Exit.")#!/usr/bin/env python3
"""
pi_yolo_detect_serial.py

Live detection on Raspberry Pi (Picamera2 or USB camera) with Ultralytics YOLO.
Sends 'metal\n' / 'glass\n' / 'paper\n' to Arduino via serial after debounce and waits for ACK.

Usage example:
  python3 pi_yolo_detect_serial.py --model runs/detect/train/weights/best.pt --source picamera0 --resolution 1280x720 --port /dev/ttyACM0

"""

import argparse
import time
import sys
import os

import cv2
import numpy as np
import serial

# Try to import Picamera2 if user wants picamera
USE_PICAMERA2 = False
try:
    from picamera2 import Picamera2
    USE_PICAMERA2 = True
except Exception:
    # not installed or not available; OK for USB cameras
    USE_PICAMERA2 = False

# ultralytics
try:
    from ultralytics import YOLO
except Exception as e:
    print("ERROR: ultralytics not available. Install with: pip3 install ultralytics")
    print("Exception:", e)
    sys.exit(1)


# ----------------------------
# CLI arguments
# ----------------------------
parser = argparse.ArgumentParser()
parser.add_argument('--model', required=True, help='Path to Ultralytics .pt model (e.g. runs/detect/train/weights/best.pt)')
parser.add_argument('--source', required=True, help='Source: usb0, usb1, picamera0, video.mp4, or image file/folder')
parser.add_argument('--resolution', default=None, help='Resolution WIDTHxHEIGHT (e.g. 1280x720). If omitted uses camera default.')
parser.add_argument('--thresh', type=float, default=0.60, help='Confidence threshold (default 0.60)')
parser.add_argument('--consec', type=int, default=3, help='Consecutive frames required (default 3)')
parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port to Arduino (default /dev/ttyACM0). Omit to run without Arduino.')
parser.add_argument('--baud', type=int, default=9600, help='Serial baud rate (default 9600)')
parser.add_argument('--ack-timeout', type=float, default=40.0, help='Seconds to wait for Arduino ACK (default 40s)')
parser.add_argument('--cooldown', type=float, default=2.0, help='Cooldown seconds between sends (default 2s)')
parser.add_argument('--show', action='store_true', help='Show OpenCV window (set if you have desktop or X forwarding).')
args = parser.parse_args()

MODEL_PATH = args.model
SOURCE = args.source
RESOLUTION = args.resolution
CONF_THRESH = args.thresh
CONSECUTIVE_FRAMES = args.consec
SERIAL_PORT = args.port if args.port.lower() != "none" else None
BAUDRATE = args.baud
ACK_TIMEOUT_SEC = args.ack_timeout
COOLDOWN_SEC = args.cooldown
SHOW_WINDOW = args.show

TARGET_CLASSES = {"metal", "glass", "paper"}  # ensure these match names in your model

ACK_KEYWORDS = ["cycle complete", "back to hold", "done", "ok"]  # keywords from Arduino printing

# ----------------------------
# Validate model file
# ----------------------------
if not os.path.exists(MODEL_PATH):
    print("ERROR: Model file not found:", MODEL_PATH)
    sys.exit(1)

# ----------------------------
# Load YOLO model
# ----------------------------
print("Loading YOLO model:", MODEL_PATH)
model = YOLO(MODEL_PATH, task='detect')

# get labels mapping
try:
    labels = model.names
except Exception:
    labels = {}

# ----------------------------
# Open serial
# ----------------------------
ser = None
if SERIAL_PORT:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        time.sleep(2.0)  # allow Arduino reset if any
        print(f"Opened serial port {SERIAL_PORT} @ {BAUDRATE}")
    except Exception as e:
        print("Warning: could not open serial port:", e)
        ser = None
else:
    print("Serial disabled (no port). Script will run but not actually send commands.")

def send_serial_line(msg: str):
    if not ser:
        print("[SIM] ->", msg.strip())
        return False
    try:
        ser.write(msg.encode())
        return True
    except Exception as e:
        print("Serial write error:", e)
        return False

def read_serial_line_nonblocking():
    if not ser:
        return None
    try:
        line = ser.readline()
        if not line:
            return None
        try:
            return line.decode(errors='replace').strip()
        except:
            return repr(line)
    except Exception as e:
        print("Serial read error:", e)
        return None

# ----------------------------
# Initialize camera/source
# ----------------------------
use_picamera = False
cap = None
picam = None

if SOURCE.startswith("usb"):
    # expect usb0, usb1 ...
    try:
        cam_idx = int(SOURCE[3:])
    except:
        print("Invalid usb source. Use usb0, usb1 etc.")
        sys.exit(1)
    cap = cv2.VideoCapture(cam_idx)
    if RESOLUTION:
        w, h = RESOLUTION.split('x')
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(w))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(h))
elif SOURCE.startswith("picamera"):
    if not USE_PICAMERA2:
        print("Picamera2 not available. Install picamera2 package or use usb camera.")
        sys.exit(1)
    use_picamera = True
    picam = Picamera2()
    if RESOLUTION:
        w, h = map(int, RESOLUTION.split('x'))
    else:
        # default resolution
        w, h = 1280, 720
    config = picam.create_preview_configuration(main={"format": 'XRGB8888', "size": (w, h)})
    picam.configure(config)
    picam.start()
elif os.path.isfile(SOURCE):
    # video or image file
    _, ext = os.path.splitext(SOURCE)
    ext = ext.lower()
    if ext in ['.jpg', '.jpeg', '.png', '.bmp']:
        cap = None
        image_mode = True
        frame_image = cv2.imread(SOURCE)
    else:
        cap = cv2.VideoCapture(SOURCE)
        if RESOLUTION:
            w, h = RESOLUTION.split('x')
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(w))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(h))
else:
    print("Unsupported source:", SOURCE)
    sys.exit(1)

# ----------------------------
# Colors for boxes
# ----------------------------
bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106),
               (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

# ----------------------------
# Main loop: detection, send, wait for ACK
# ----------------------------
consecutive = 0
last_label = None
last_sent_time = 0.0
waiting_for_ack = False
ack_start_time = 0.0

print("Starting capture. Press Ctrl+C to quit or close window if shown.")

try:
    while True:
        # poll serial to print any incoming lines (so they are visible while waiting)
        sline = read_serial_line_nonblocking()
        if sline:
            print("[Arduino] ", sline)
            if waiting_for_ack:
                low = sline.lower()
                for kw in ACK_KEYWORDS:
                    if kw in low:
                        waiting_for_ack = False
                        print("ACK detected from Arduino (keyword):", kw)
                        consecutive = 0
                        last_label = None
                        break

        # capture frame
        if use_picamera:
            frame_bgra = picam.capture_array()
            frame = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)
        elif cap is not None:
            ret, frame = cap.read()
            if not ret:
                print("Frame not read. Exiting.")
                break
        else:
            # image file mode
            frame = frame_image.copy()

        # optionally resize (already set earlier)
        # run detection only if not waiting for ack
        if not waiting_for_ack:
            # model expects BGR numpy frames directly
            results = model(frame, verbose=False)
            detections = results[0].boxes

            # draw boxes and find best target
            chosen = None
            best_conf = 0.0
            object_count = 0

            for i in range(len(detections)):
                xyxy_tensor = detections[i].xyxy.cpu()
                xyxy = xyxy_tensor.numpy().squeeze()
                xmin, ymin, xmax, ymax = xyxy.astype(int)

                classidx = int(detections[i].cls.item())
                classname = labels.get(classidx, str(classidx)) if isinstance(labels, dict) else labels[classidx]
                conf = float(detections[i].conf.item())

                if conf >= CONF_THRESH:
                    color = bbox_colors[classidx % len(bbox_colors)]
                    cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                    label_text = f"{classname}: {conf:.2f}"
                    cv2.putText(frame, label_text, (xmin, max(ymin, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)
                    object_count += 1

                    lab = classname.lower().strip()
                    if lab in TARGET_CLASSES and conf > best_conf:
                        best_conf = conf
                        chosen = (lab, conf, (xmin, ymin, xmax, ymax))

            # show overlay information
            cv2.putText(frame, f"Objects: {object_count}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

            # debounce / confirmation
            if chosen:
                lab, conf, _ = chosen
                if lab == last_label:
                    consecutive += 1
                else:
                    last_label = lab
                    consecutive = 1

                cv2.putText(frame, f"Candidate: {lab} ({consecutive}/{CONSECUTIVE_FRAMES}) conf={conf:.2f}", (10,60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,200), 2)

                if consecutive >= CONSECUTIVE_FRAMES:
                    now = time.time()
                    if now - last_sent_time >= COOLDOWN_SEC:
                        msg = lab + "\n"
                        ok = send_serial_line(msg)
                        if ok:
                            print("Sent to Arduino:", lab)
                        else:
                            print("Simulated send (no serial):", lab)
                        last_sent_time = now
                        waiting_for_ack = True
                        ack_start_time = time.time()
                        consecutive = 0
                        last_label = None
                        cv2.putText(frame, f"SENT: {lab}", (10,100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 3)
                    else:
                        cd_rem = COOLDOWN_SEC - (now - last_sent_time)
                        cv2.putText(frame, f"Cooldown: {cd_rem:.1f}s", (10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,200), 2)
            else:
                consecutive = 0
                last_label = None
        else:
            # waiting for ack - overlay and timeout
            elapsed = time.time() - ack_start_time
            cv2.putText(frame, f"Waiting for Arduino ACK... {int(elapsed)}s", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,200,255), 2)
            if elapsed > ACK_TIMEOUT_SEC:
                print(f"ACK timeout after {ACK_TIMEOUT_SEC}s - resuming detection.")
                waiting_for_ack = False
                consecutive = 0
                last_label = None

        # show window if requested
        if SHOW_WINDOW:
            cv2.imshow("YOLO Live (press q to quit)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("User requested exit.")
                break
        else:
            # If no window, allow short sleep to avoid 100% CPU loop
            time.sleep(0.01)

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    print("Cleaning up...")
    try:
        if use_picamera and picam:
            picam.stop()
        if cap is not None:
            cap.release()
    except:
        pass
    if ser:
        try:
            ser.close()
        except:
            pass
    if SHOW_WINDOW:
        cv2.destroyAllWindows()
    print("Exit.")