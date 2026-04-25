import cv2
import numpy as np
from picamera2 import Picamera2
import serial
import time
import pigpio

# =========================
# GPIO / SERVO SETUP
# =========================
servo1 = 12
servo2 = 13
servo3 = 18
servo4 = 19

curr1 = 0
curr2 = 0
curr3 = 0
curr4 = 0

pi = pigpio.pi()

if not pi.connected:
    print("Could not connect to pigpiod. Did you run 'sudo pigpiod'?")
    exit()

def set_angle(pin, angle):
    if 0 <= angle <= 180:
        pulsewidth = 500 + (angle / 180) * 2000
        pi.set_servo_pulsewidth(pin, pulsewidth)
        
def stop_servo(pin):
    pi.set_servo_pulsewidth(pin, 0)  # 0 = stop sending pulses

def start(a1, a2, a3, a4):
    global curr1, curr2, curr3, curr4
    set_angle(servo1, a1)
    set_angle(servo3, a3)
    set_angle(servo2, a2)
    set_angle(servo4, a4)
    curr1 = a1
    curr2 = a2
    curr3 = a3
    curr4 = a4
    time.sleep(1.5)
    stop_servo(servo1)
    stop_servo(servo2)
    stop_servo(servo3)
    stop_servo(servo4)

def move(a1, a2, a3, a4):
    global curr1, curr2, curr3, curr4
    for i in range(10):
        b1 = curr1 + (a1 - curr1) * i / 9
        b2 = curr2 + (a2 - curr2) * i / 9
        b3 = curr3 + (a3 - curr3) * i / 9
        b4 = curr4 + (a4 - curr4) * i / 9
        set_angle(servo1, b1)
        set_angle(servo3, b3)
        set_angle(servo2, b2)
        set_angle(servo4, b4)
        time.sleep(0.1)
    curr1 = a1
    curr2 = a2
    curr3 = a3
    curr4 = a4
    stop_servo(servo1)
    stop_servo(servo2)
    stop_servo(servo3)
    stop_servo(servo4)

# =========================
# LOAD MODEL
# =========================
net = cv2.dnn.readNetFromONNX("yolov8n.onnx")

# =========================
# CAMERA SETUP
# =========================
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (320, 240)}
))
picam2.start()

ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
time.sleep(2)

# =========================
# DETECTION SETTINGS
# =========================
INPUT_SIZE = 640
CONF_THRESHOLD = 0.38      # raised from 0.45 to reduce false positives
NMS_THRESHOLD = 0.35
MIN_BOX_AREA = 400          # ignore tiny noisy detections (px²)

# =========================
# SHARED DETECTION FUNCTION
# =========================
def run_detection(frame):
    """
    Runs YOLOv8 ONNX inference on frame.
    Returns (detected: bool, annotated_frame).
    Fixes:
      - NMS applied ONCE after all detections collected
      - Minimum box area filter
      - Safe index unpacking for all OpenCV versions
    """
    h, w, _ = frame.shape
    blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (INPUT_SIZE, INPUT_SIZE), swapRB=True)
    net.setInput(blob)
    outputs = net.forward()

    # YOLOv8 ONNX output reshape
    if outputs.shape[1] == 84:
        outputs = np.transpose(outputs, (0, 2, 1))

    detections = outputs[0]
    boxes = []
    scores = []

    # --- Collect ALL detections first, THEN apply NMS ---
    for det in detections:
        class_scores = det[4:]
        score = float(np.max(class_scores))

        if score < CONF_THRESHOLD:
            continue

        cx, cy, bw, bh = det[0:4]

        # Scale from model input size to frame size
        cx *= w / INPUT_SIZE
        cy *= h / INPUT_SIZE
        bw *= w / INPUT_SIZE
        bh *= h / INPUT_SIZE

        # Filter out tiny boxes (noise / distant non-objects)
        if bw * bh < MIN_BOX_AREA:
            continue

        x1 = int(cx - bw / 2)
        y1 = int(cy - bh / 2)
        boxes.append([x1, y1, int(bw), int(bh)])
        scores.append(score)

    detected = False

    if len(boxes) > 0:
        # Apply NMS once on the full set of boxes
        indices = cv2.dnn.NMSBoxes(boxes, scores, CONF_THRESHOLD, NMS_THRESHOLD)

        # Flatten indices safely (OpenCV 4.x returns nested, 4.5+ returns flat)
        if len(indices) > 0:
            indices = [i[0] if isinstance(i, (list, tuple, np.ndarray)) else i for i in indices]
            detected = True

            for i in indices:
                x, y, bw, bh = boxes[i]
                cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                label = f"{scores[i]:.2f}"
                cv2.putText(frame, label, (x, max(y - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return detected, frame

# =========================
# STATE MACHINE INIT
# =========================
state = "INIT"
state_time = time.time()
detect_start_time = 0
ring = 0   # 0 = ring position, 1 = pin position

# =========================
# MAIN LOOP
# =========================
while True:
    frame = picam2.capture_array()

    if state == "INIT":
        print("Go initial")
        ser.write(b'0\n')
        start(143, 70, 65, 60)
        time.sleep(2)
        state = "WAIT_INIT"
        state_time = time.time()

    elif state == "WAIT_INIT":
        if time.time() - state_time > 4:
            state = "DETECT_INITIAL"
            detect_start_time = time.time()  # reset timer on every entry

    elif state == "DETECT_INITIAL":
        print("detecting")
        detected = False
        detected, frame = run_detection(frame)

        if detected:
            print("Assembled object detected → PICK")
            ser.write(b'2\n')
            move(149, 80, 85, 80)
            time.sleep(1)
            move(149, 69, 95, 105)
            time.sleep(1)
            ser.write(b'1\n')
            state = "PICK_WAIT1"
            state_time = time.time()

        elif time.time() - detect_start_time > 3:
            if ring == 0:
                print("No object → go RING position")
                start(149, 90, 60, 95)
                time.sleep(2)
                start(100, 90, 60, 95)
                time.sleep(2)
                move(100, 75, 100, 101)
                start(100, 75, 100, 101)
                time.sleep(1)
                start(109, 75, 100, 101)
                time.sleep(1)
                ser.write(b'RING\n')
                state = "WAIT_MOVE1"
                state_time = time.time()
            else:
                print("No object → go PIN position")
                start(149, 90, 60, 95)
                time.sleep(2)
                start(94, 90, 60, 95)
                time.sleep(2)
                move(92, 75, 95, 97)
                start(92, 75, 95, 99)
                time.sleep(1)
                ser.write(b'PIN\n')
                state = "WAIT_MOVE2"
                state_time = time.time()

    elif state == "WAIT_MOVE1":
        if time.time() - state_time > 4:
            state = "DETECT_SECOND"
            detect_start_time = time.time()  # reset timer on entry

    elif state == "WAIT_MOVE2":
        if time.time() - state_time > 4:
            state = "DETECT_THIRD"
            detect_start_time = time.time()  # reset timer on entry

    elif state == "DETECT_SECOND":
        print("detecting")
        detected = False
        detected, frame = run_detection(frame)

        if detected:
            print("Object found → PICK (ring)")
            ser.write(b'2\n')
            start(109, 75, 100, 101)
            time.sleep(1)
            start(109, 71, 104, 110)
            time.sleep(1)
            ser.write(b'1\n')
            state = "PICK_WAIT2"
            state_time = time.time()

        elif time.time() - detect_start_time > 3:
            print("Still no object → back to INIT")
            move(110, 84, 75, 95)
            start(110, 84, 75, 95)
            time.sleep(1)
            move(149, 84, 75, 95)
            time.sleep(2)
            state = "INIT"

    elif state == "DETECT_THIRD":
        print("detecting")
        detected = False
        detected, frame = run_detection(frame)

        if detected:
            print("Object found → PICK (pin)")
            ser.write(b'2\n')
            start(94, 75, 95, 108)
            time.sleep(1)
            start(94, 78, 110, 112)
            time.sleep(1)
            start(94, 70, 110, 112)
            time.sleep(1)
            ser.write(b'1\n')
            state = "PICK_WAIT3"
            state_time = time.time()

        elif time.time() - detect_start_time > 3:
            print("Still no object → back to INIT")
            move(94, 84, 75, 95)
            start(94, 84, 75, 95)
            time.sleep(1)
            move(149, 84, 75, 95)
            time.sleep(2)
            state = "INIT"

    elif state == "PICK_WAIT1":
        if time.time() - state_time > 4:
            print("Go DROP (assembled)")
            start(149, 69, 110, 105)
            time.sleep(1)
            start(149, 105, 110, 95)
            time.sleep(1)
            move(180, 100, 110, 95)
            time.sleep(1)
            move(180, 76, 92, 86)
            start(180, 76, 92, 86)
            time.sleep(1)
            ser.write(b'2\n')
            time.sleep(2)
            move(180, 84, 92, 95)
            state = "DROP_WAIT"
            state_time = time.time()

    elif state == "PICK_WAIT2":
        if time.time() - state_time > 4:
            print("Go DROP (ring)")
            start(110, 84, 100, 120)
            time.sleep(2)
            start(110, 84, 70, 95)
            time.sleep(2)
            move(77, 84, 75, 95)
            start(77, 84, 75, 95)
            time.sleep(1)
            ser.write(b'2\n')
            ring = 1
            state = "DROP_WAIT"
            state_time = time.time()

    elif state == "PICK_WAIT3":
        if time.time() - state_time > 4:
            print("Go DROP (pin)")
            start(94, 90, 110, 110)
            time.sleep(1)
            start(94, 90, 70, 95)
            time.sleep(1)
            move(77, 86, 75, 85)
            start(77, 86, 75, 85)
            time.sleep(1)
            ser.write(b'2\n')
            ring = 0
            state = "DROP_WAIT"
            state_time = time.time()

    elif state == "DROP_WAIT":
        if time.time() - state_time > 4:
            print("Cycle complete → INIT")
            ser.write(b'0\n')
            move(149, 86, 75, 85)
            start(149, 86, 75, 85)
            state = "INIT"

    # Display
    cv2.imshow("ONNX Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# =========================
# CLEANUP
# =========================
cv2.destroyAllWindows()
picam2.stop()
pi.stop()