from flask import Flask, Response, request, jsonify, session
from flask_cors import CORS
import depthai as dai
import cv2
import serial
import time
import threading
import numpy as np
import blobconverter
from pymavlink import mavutil

app = Flask(__name__)
CORS(app, supports_credentials=True)
app.secret_key = 'supersecretkey'

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

# Function to read data from Arduino
def read_serial_data():
    if arduino.in_waiting > 0:
        line = arduino.readline().decode('utf-8').rstrip()
        return line
    return None

# Function to read data from Pixhawk
def read_pixhawk_data():
    while True:
        msg = pixhawk_connection.recv_match(type='RC_CHANNELS', blocking=True)
        if msg:
            task_channel = getattr(msg, 'chan7_raw', None)
            if task_channel is not None:
                arduino.write(f"{task_channel}\n".encode())
                print(f"Sent task channel data to Arduino: {task_channel}")

# Function to create the depthai pipeline with detection
def create_pipeline_with_detection():
    nnPath = blobconverter.from_zoo(name="yolov8n_coco_640x352", zoo_type="depthai", shaves=6)

    pipeline = dai.Pipeline()

    camRgb = pipeline.create(dai.node.ColorCamera)
    detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    nnOut = pipeline.create(dai.node.XLinkOut)

    xoutRgb.setStreamName("rgb")
    nnOut.setStreamName("nn")

    camRgb.setPreviewSize(640, 352)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.setFps(40)

    detectionNetwork.setConfidenceThreshold(0.5)
    detectionNetwork.setNumClasses(80)
    detectionNetwork.setCoordinateSize(4)
    detectionNetwork.setAnchors([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319])
    detectionNetwork.setAnchorMasks({"side26": [1, 2, 3], "side13": [3, 4, 5]})
    detectionNetwork.setIouThreshold(0.5)
    detectionNetwork.setBlobPath(nnPath)
    detectionNetwork.setNumInferenceThreads(2)
    detectionNetwork.input.setBlocking(False)

    camRgb.preview.link(detectionNetwork.input)
    detectionNetwork.passthrough.link(xoutRgb.input)
    detectionNetwork.out.link(nnOut.input)

    return pipeline

# Initialize the pipeline and device
pipeline = create_pipeline_with_detection()
device = dai.Device(pipeline)
rgb_queue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
nn_queue = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

# Connect to Pixhawk
pixhawk_connection = connect_pixhawk()
pixhawk_connection.mav.request_data_stream_send(
    pixhawk_connection.target_system,
    pixhawk_connection.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
    5,
    1
)

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

# Helper functions for detection
def frameNorm(frame, bbox):
    normVals = np.full(len(bbox), frame.shape[0])
    normVals[::2] = frame.shape[1]
    return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

def displayFrame(frame, detections):
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
        
    return frame

# Function to generate frames for video feed with detection
def generate_frames():
    while True:
        inRgb = rgb_queue.get()
        inDet = nn_queue.get()

        if inRgb is not None:
            frame = inRgb.getCvFrame()
            if inDet is not None:
                detections = inDet.detections
                frame = displayFrame(frame, detections)
            
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

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

@app.route('/video_feed')
def video_feed():
    if not session.get('logged_in'):
        print("Unauthorized access to video feed")
        return Response(status=401)
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/sensor_data')
def sensor_data():
    if not session.get('logged_in'):
        print("Unauthorized access to sensor_data")
        return Response(status=401)

    data = read_serial_data()
    if data:
        sensor_data = data.split(',')
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

    return jsonify({
        'error': 'No sensor data received from Arduino'
    }), 500

if __name__ == "__main__":
    threading.Thread(target=read_pixhawk_data).start()
    app.run(host='0.0.0.0', port=5000, debug=False)
