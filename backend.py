import eventlet
eventlet.monkey_patch()

from flask import Flask, request, jsonify, session
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import depthai as dai
import cv2
import base64
import numpy as np
import blobconverter
import serial
from pymavlink import mavutil
import time

app = Flask(__name__)
CORS(app, supports_credentials=True)
app.secret_key = 'supersecretkey'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

# Establish serial connection to Arduino Mega for distance sensors
arduino = serial.Serial('/dev/ttyS0', 115200, timeout=1)
arduino.flush()

# Function to connect to Pixhawk
def connect_pixhawk():
    ports = ['/dev/ttyACM0', '/dev/ttyACM1']
    for port in ports:
        try:
            connection = mavutil.mavlink_connection(port, baud=57600)
            connection.wait_heartbeat()
            print(f"Connected to Pixhawk on {port}")
            return connection
        except Exception as e:
            print(f"Failed to connect to Pixhawk on {port}: {e}")
    raise Exception("Could not connect to Pixhawk on any known port")

# Establish Pixhawk connection
pixhawk_connection = connect_pixhawk()

pixhawk_connection.mav.request_data_stream_send(
    pixhawk_connection.target_system,
    pixhawk_connection.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
    5,
    1
)

# Function to disable pre-arm checks
def disable_pre_arm_checks():
    print("Disabling specific pre-arm checks...")
    pixhawk_connection.mav.param_set_send(
        pixhawk_connection.target_system,
        pixhawk_connection.target_component,
        b'ARMING_CHECK',
        1,  # 1 Enables / 0 Disables all pre-arm checks
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    # Wait for the parameter to be set
    eventlet.sleep(2)

disable_pre_arm_checks()

# Function to set stabilize mode
def set_stabilize_mode():
    mode_id = pixhawk_connection.mode_mapping().get('STABILIZE')
    if mode_id is None:
        print("STABILIZE mode not available")
        return False

    pixhawk_connection.mav.set_mode_send(
        pixhawk_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    # Check if the mode has been set successfully by reading the current mode
    start_time = time.monotonic()
    while time.monotonic() - start_time < 5:  # 5 seconds timeout
        ack_msg = pixhawk_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if ack_msg:
            current_mode = mavutil.mode_string_v10(ack_msg)
            print(f"Current mode: {current_mode}")
            if current_mode == 'STABILIZE':
                print("STABILIZE mode set successfully")
                return True
    print("Timeout while setting STABILIZE mode")
    return False

set_stabilize_mode()

# Function to arm the Pixhawk
def arm_command():
    print("Arming Pixhawk...")
    pixhawk_connection.mav.command_long_send(
        pixhawk_connection.target_system,
        pixhawk_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)

# Function to disarm the Pixhawk
def disarm_command():
    print("Disarming Pixhawk...")
    pixhawk_connection.mav.command_long_send(
        pixhawk_connection.target_system,
        pixhawk_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0)

# Function to confirm disarm status
def confirm_disarm():
    print("Confirming disarm status...")
    start_time = time.monotonic()
    while time.monotonic() - start_time < 5:  # 5 seconds timeout
        ack_msg = pixhawk_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if ack_msg:
            if ack_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED == 0:
                print("Pixhawk disarmed successfully")
                return True
    print("Failed to disarm Pixhawk")
    return False

armed = False
autonomous_mode = False
last_person_time = None

# Function to read data from Arduino
def read_serial_data():
    buffer = ""
    while True:
        if arduino.in_waiting > 0:
            buffer += arduino.read(arduino.in_waiting).decode('utf-8')
            if '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                return line
        eventlet.sleep(0.01)

# Function to read data from Pixhawk
def read_pixhawk_data():
    global armed, autonomous_mode
    while True:
        try:
            msg = pixhawk_connection.recv_match(type='RC_CHANNELS', blocking=True)
            if msg:
                task_channel = getattr(msg, 'chan7_raw', None)
                if task_channel and task_channel >= 1700:
                    autonomous_mode = True
                else:
                    autonomous_mode = False

                if task_channel and 1400 <= task_channel <= 1600:
                    mode_channel = getattr(msg, 'chan8_raw', None)
                    if mode_channel is not None:
                        command = ""
                        if mode_channel >= 1900:
                             if not armed:
                                 command = "0"
                                 eventlet.sleep(2)
                                 arm_command()
                                 armed = True
                        elif mode_channel <= 1200:
                            if armed:
                                 disarm_command()
                                 eventlet.sleep(3)
                                 armed = not confirm_disarm()
                            command = "1"
                            up_down_channel = getattr(msg, 'chan2_raw', None)
                            left_right_channel = getattr(msg, 'chan1_raw', None)
                            if up_down_channel is not None and left_right_channel is not None:
                                if up_down_channel <= 1475:
                                    num = (up_down_channel - 1000) / 100
                                    if num >= 0 and num <= 1:
                                        speed = "3"
                                    elif num > 1 and num <= 2:
                                        speed = "2"
                                    elif num > 2 and num <= 3:
                                        speed = "1"
                                    else:
                                        speed = "0"
                                    command += "2" + speed
                                elif up_down_channel >= 1525:
                                    num = (up_down_channel - 1525) / 100
                                    if num >= 0 and num <= 1:
                                        speed = "0"
                                    elif num > 1 and num <= 2:
                                        speed = "1"
                                    elif num > 2 and num <= 3:
                                        speed = "2"
                                    else:
                                        speed = "3"
                                    command += "1" + speed
                                else:
                                    if left_right_channel <= 1475:
                                        num = (left_right_channel - 1000) / 100
                                        if num >= 0 and num <= 1:
                                            speed = "3"
                                        elif num > 1 and num <= 2:
                                            speed = "2"
                                        elif num > 2 and num <= 3:
                                            speed = "1"
                                        else:
                                            speed = "0"
                                        command += "3" + speed
                                    elif left_right_channel >= 1525:
                                        num = (left_right_channel - 1525) / 100
                                        if num >= 0 and num <= 1:
                                            speed = "0"
                                        elif num > 1 and num <= 2:
                                            speed = "1"
                                        elif num > 2 and num <= 3:
                                            speed = "2"
                                        else:
                                            speed = "3"
                                        command += "4" + speed
                                    else:
                                        command += "0"

                        if command:
                            print(command)
                            arduino.write((command + '\n').encode())
                            eventlet.sleep(0.01)
        except Exception as e:
            print(f"Error in read_pixhawk_data: {e}")
            eventlet.sleep(1)

labelMap = [
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
    "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
    "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

def get_depthai_pipeline():
    nnPath = blobconverter.from_zoo(name="yolov8n_coco_640x352", zoo_type="depthai", shaves=5)

    pipeline = dai.Pipeline()

    camRgb = pipeline.create(dai.node.ColorCamera)
    spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutNN = pipeline.create(dai.node.XLinkOut)
    xoutDepth = pipeline.create(dai.node.XLinkOut)

    xoutRgb.setStreamName("rgb")
    xoutNN.setStreamName("detections")
    xoutDepth.setStreamName("depth")

    camRgb.setPreviewSize(640, 352)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.setFps(10)

    spatialDetectionNetwork.setBlobPath(nnPath)
    spatialDetectionNetwork.setConfidenceThreshold(0.5)
    spatialDetectionNetwork.setNumClasses(80)
    spatialDetectionNetwork.setCoordinateSize(4)
    spatialDetectionNetwork.setAnchors([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319])
    spatialDetectionNetwork.setAnchorMasks({"side26": [1, 2, 3], "side13": [3, 4, 5]})
    spatialDetectionNetwork.setIouThreshold(0.5)
    spatialDetectionNetwork.input.setBlocking(False)
    spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
    spatialDetectionNetwork.setDepthLowerThreshold(100)
    spatialDetectionNetwork.setDepthUpperThreshold(5000)

    camRgb.preview.link(spatialDetectionNetwork.input)
    spatialDetectionNetwork.passthrough.link(xoutRgb.input)
    spatialDetectionNetwork.out.link(xoutNN.input)

    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
    stereo.setSubpixel(True)

    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    stereo.depth.link(spatialDetectionNetwork.inputDepth)
    spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

    return pipeline

pipeline = get_depthai_pipeline()
device = dai.Device(pipeline)
rgb_queue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
nn_queue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
depth_queue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

def frameNorm(frame, bbox):
    normVals = np.full(len(bbox), frame.shape[0])
    normVals[::2] = frame.shape[1]
    return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

def displayFrame(frame, detections):
    global last_person_time
    
    person_detected = False
    for detection in detections:
        bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
        
        # Draw the bounding box
        color = (0, 255, 0)  # Green color for the bounding box
        cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        
        # Prepare text labels
        label = labelMap[detection.label]
        confidence = f"{int(detection.confidence * 100)}%"
        text = f"{label} {confidence}"
        
        # Get text size and background size
        (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        text_bg_coords = ((bbox[0], bbox[1] - h - 10), (bbox[0] + w, bbox[1]))
        
        # Draw text background rectangle
        cv2.rectangle(frame, text_bg_coords[0], text_bg_coords[1], color, -1)
        
        # Draw text
        cv2.putText(frame, text, (bbox[0], bbox[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
        
        if label == "person":
            last_person_time = time.monotonic()
            person_detected = True
            if autonomous_mode:
                autonomous(detection)
                break

    if autonomous_mode and not person_detected:
        if last_person_time is None or (time.monotonic() - last_person_time) >= 2:
            print("No person detected for 2 seconds, staying")
            arduino.write(("10" + '\n').encode())

    return frame

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    emit('hello', 'world')

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

def generate_frames():
    while True:
        try:
            inRgb = rgb_queue.get()
            inDet = nn_queue.get()

            if inRgb is not None:
                frame = inRgb.getCvFrame()
                if inDet is not None:
                    detections = inDet.detections
                    frame = displayFrame(frame, detections)

                # Encode the frame to JPEG format
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
                ret, buffer = cv2.imencode('.jpg', frame, encode_param)
                if not ret:
                    continue  # Skip if encoding failed

                # Send the raw bytes over SocketIO without specifying 'binary=True'
                socketio.emit('video_frame', buffer.tobytes())
                eventlet.sleep(0.01)
        except Exception as e:
            print(f"Error in generate_frames: {e}")
            eventlet.sleep(1)




def autonomous(detection):
    global last_state, last_person_time, middle_time

    person_center = (detection.xmin + detection.xmax) / 2
    frame_center = 0.5
    threshold = 0.16
    current_state = ""

    if person_center < frame_center - threshold:
        current_state = "left"
    elif person_center > frame_center + threshold:
        current_state = "right"
    else:
        current_state = "middle"

    if current_state == "left":
        print("Detected person to the left")
        arduino.write(("134" + '\n').encode())
    elif current_state == "right":
        print("Detected person to the right")
        arduino.write(("144" + '\n').encode())
    elif current_state == "middle":
        # Calculate distance to the person
        person_depth = detection.spatialCoordinates.z
        print(f"Person depth: {person_depth} mm")
        if person_depth > 1100:  # 700 mm or 70 cm
            print("Person centered, moving forward")
            arduino.write(("111" + '\n').encode())  # Example forward command
        elif person_depth <= 1100:  # Person is too close
            print("Person too close, staying")
            arduino.write(("10" + '\n').encode())

@app.route('/login', methods=['POST'])
def login():
    data = request.json
    username = data.get('username')
    password = data.get('password')

    if username == 'quest' and password == 'iosub':
        session['logged_in'] = True
        print("Login successful")
        return jsonify({'success': True}), 200
    else:
        print("Login failed")
        return jsonify({'success': False}), 401

@app.route('/sensor_data')
def sensor_data():
    data = read_serial_data()
    if data:
        sensor_data = data.split(',')
        if len(sensor_data) != 6:
            return jsonify({'error': 'Invalid sensor data format'}), 500
        try:
            front_distance = int(sensor_data[0])
            right_distance = int(sensor_data[1])
            left_distance = int(sensor_data[2])
            back_distance = int(sensor_data[3])
            temperature = float(sensor_data[4])
            humidity = float(sensor_data[5])

            return jsonify({
                'front': front_distance,
                'right': right_distance,
                'left': left_distance,
                'back': back_distance,
                'temperature': temperature,
                'humidity': humidity
            })
        except ValueError as e:
            return jsonify({'error': f'Invalid sensor data values: {e}'}), 500

    return jsonify({
        'error': 'No sensor data received from Arduino'
    }), 500

@app.route('/')
def index():
    return "WebSocket server is running"

if __name__ == "__main__":
    eventlet.spawn(read_pixhawk_data)
    eventlet.spawn(generate_frames)
    socketio.run(app, host='0.0.0.0', port=5000)
