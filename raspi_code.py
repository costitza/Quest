import io
import socket
import struct
import time
import signal
import atexit
import depthai as dai
import cv2
import blobconverter
from gpiozero import DistanceSensor
import serial
from pymavlink import mavutil

COCO_CLASSES = [
    "persoana", "bicicleta", "masina", "motocicleta", "avion", "autobuz", "tren",
    "camion", "barca", "semafor", "hidrante", "semn de oprire",
    "parcmetru", "banca", "pasare", "pisica", "caine", "cal", "oaie",
    "vaca", "elefant", "urs", "zebra", "girafa", "rucsac", "umbrela",
    "geanta de mana", "cravata", "valiza", "frisbee", "schiuri", "snowboard",
    "minge de sport", "zmeu", "bata de baseball", "manusa de baseball", "skateboard",
    "placa de surf", "racheta de tenis", "sticla", "pahar de vin", "cana", "furculita",
    "cutit", "lingura", "bol", "banana", "mar", "sandwich", "portocala",
    "broccoli", "morcov", "hot dog", "pizza", "gogoasa", "tort", "scaun",
    "canapea", "planta in ghiveci", "pat", "masa de dining", "toaleta", "televizor",
    "laptop", "mouse", "telecomanda", "tastatura", "telefon mobil", "cuptor cu microunde",
    "cuptor", "prajitor de paine", "chiuveta", "frigider", "carte", "ceas", "vaza",
    "foarfece", "ursulet de plus", "uscator de par", "periuta de dinti"
]

def wait_for_wifi(timeout=60):
    start_time = time.time()
    while True:
        try:
            socket.create_connection(("8.8.8.8", 53))
            print("Connected to the internet")
            return
        except OSError:
            if time.time() - start_time > timeout:
                raise Exception("Failed to connect to Wi-Fi within the timeout period")
            print("Waiting for Wi-Fi...")
            time.sleep(1)

def connect_arduino():
    ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/serial0', '/dev/ttyS0']
    for port in ports:
        try:
            arduino = serial.Serial(port, 115200, timeout=1)
            print(f"Connected to Arduino on {port}")
            return arduino
        except serial.SerialException:
            print(f"Failed to connect to Arduino on {port}")
    raise Exception("Could not connect to Arduino on any known port")

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


wait_for_wifi()

arduino = connect_arduino()

pixhawk_connection = connect_pixhawk()

ultrasonic = DistanceSensor(echo=5, trigger=6)


pixhawk_connection.mav.request_data_stream_send(
    pixhawk_connection.target_system,    
    pixhawk_connection.target_component, 
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 
    5, 
    1  
)

def create_pipeline(blob_path):
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.createColorCamera()
    yolo_nn = pipeline.createYoloDetectionNetwork()
    xout_rgb = pipeline.createXLinkOut()
    xout_nn = pipeline.createXLinkOut()

    xout_rgb.setStreamName("rgb")
    xout_nn.setStreamName("detections")

    cam_rgb.setPreviewSize(640, 352)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    yolo_nn.setConfidenceThreshold(0.5)
    yolo_nn.setNumClasses(80)
    yolo_nn.setCoordinateSize(4)
    yolo_nn.setAnchors([
        10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319
    ])
    yolo_nn.setAnchorMasks({
        "side26": [0, 1, 2],
        "side13": [3, 4, 5]
    })
    yolo_nn.setIouThreshold(0.5)
    yolo_nn.setBlobPath(blob_path)

    cam_rgb.preview.link(yolo_nn.input)
    yolo_nn.passthrough.link(xout_rgb.input)
    yolo_nn.out.link(xout_nn.input)

    return pipeline

def run_yolo():
    model_path = blobconverter.from_zoo(name="yolov8n_coco_640x352", zoo_type="depthai", shaves=6)
    pipeline = create_pipeline(model_path)

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('192.168.146.80', 8000))
    connection = client_socket.makefile('wb')

    def stop_device():
        device.close()
        connection.close()
        client_socket.close()
    
    signal.signal(signal.SIGINT, lambda sig, frame: stop_device())
    signal.signal(signal.SIGTERM, lambda sig, frame: stop_device())
    atexit.register(stop_device)

    with dai.Device(pipeline) as device:
        
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        q_nn = device.getOutputQueue(name="detections", maxSize=4, blocking=False)

        start = time.time()
        last_person_time = time.time()
        middle_time = time.time()
        last_state = ""
        
        while True:
            in_rgb = q_rgb.tryGet()
            in_nn = q_nn.tryGet()

            person_detected = False
            current_state = ""
            no_person_printed = False

            msg = pixhawk_connection.recv_match(type='RC_CHANNELS', blocking=True)
            if msg:
                task_channel = getattr(msg, 'chan7_raw', 0)

            if in_rgb is not None:
                frame = in_rgb.getCvFrame()

                if in_nn is not None:
                    detections = in_nn.detections

                    for detection in detections:
                        x1 = int(detection.xmin * frame.shape[1])
                        y1 = int(detection.ymin * frame.shape[0])
                        x2 = int(detection.xmax * frame.shape[1])
                        y2 = int(detection.ymax * frame.shape[0])
                        label_id = detection.label
                        label = COCO_CLASSES[label_id] 
                        confidence = detection.confidence

                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

                        label_text = f"{label}: {confidence:.2f}"
                        (w, h), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                        cv2.rectangle(frame, (x1, y1 - 20), (x1 + w, y1), (0, 255, 0), -1)
                        cv2.putText(frame, label_text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

                        if task_channel >= 1700:
                            if label == "persoana":
                                person_detected = True
                                last_person_time = time.time() 
                                no_person_printed = False 
                                person_center = (x1 + x2) // 2
                                frame_center = frame.shape[1] // 2
                                threshold = frame.shape[1] // 6 

                                if person_center < frame_center - threshold:
                                    current_state = "left"
                                elif person_center > frame_center + threshold:
                                    current_state = "right"
                                else:
                                    current_state = "middle"
                                    distance = ultrasonic.distance * 100 
                                    print(f"Distance: {distance:.2f} cm")
                                    
                                    distance_text = f"{distance:.2f} cm"
                                    (dw, dh), _ = cv2.getTextSize(distance_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                                    dx = x1 + (x2 - x1 - dw) // 2
                                    dy = y1 + (y2 - y1 + dh) // 2
                                    cv2.rectangle(frame, (dx - 2, dy - dh - 2), (dx + dw + 2, dy + 2), (0, 255, 0), -1)
                                    cv2.putText(frame, distance_text, (dx, dy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                                    if current_state == last_state and time.time() - middle_time > 2:
                                        if distance == 100:
                                             print("robot going forward")
                                             print(114)
                                             arduino.write(("114"+'\n').encode())
                                        else:
                                             print("robot stoped")
                                             print(10)
                                             arduino.write(("10"+'\n').encode())
                                    elif current_state != last_state:
                                         middle_time = time.time()
                                    
                if task_channel >= 1700:
                    if not person_detected:
                        if time.time() - last_person_time > 1 and not no_person_printed:
                            current_state = "no_person"
                            print("No person detected")
                            print(10)
                            arduino.write(("10" + '\n').encode())
                            print("sent 10 to arduino")
                            no_person_printed = True
                            last_state = "no person"

                    if person_detected and current_state != last_state:
                        if current_state == "left":
                            print("Detected person to the left")
                            print(134)
                            arduino.write(("134" + '\n').encode())
                            print("sent 134 to arduino")
                        elif current_state == "right":
                            print("Detected person to the right")
                            print(144)
                            arduino.write(("144" + '\n').encode())
                            print("sent 144 to arduino")
                        elif current_state == "middle":
                            middle_time = time.time()
                            print("Detected person in the middle")
                            print(10)
                            arduino.write(("10" + '\n').encode())
                            print("sent 10 to arduino")
                        
                    last_state = current_state

                result, frame = cv2.imencode('.jpg', frame)
                if not result:
                    print("Failed to encode frame")
                    break

                data = frame.tobytes()
                size = len(data)

                connection.write(struct.pack("<L", size) + data)
                connection.flush()
                
                if task_channel >=1400 and task_channel <=1600:
                    mode_channel = getattr(msg, 'chan8_raw', None)
                    
                    if mode_channel is not None:
                        command = ""
                        if mode_channel >= 1900:
                            command = "0"  
                        elif mode_channel <= 1200:
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
                                    num = (up_down_channel - 1600) / 100
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
                                        num = (left_right_channel - 1600) / 100
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
                        print(f"Command to Arduino: {command}")
                        arduino.write((command + '\n').encode())
                        print(f"Sent command to Arduino: {command} \n")

        connection.write(struct.pack('<L', 0))

if __name__ == "__main__":
    run_yolo()


