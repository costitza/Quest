import io
import socket
import struct
import cv2
import numpy as np


def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 1))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip


# Set up the socket server
socketserver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_ip = get_local_ip()  # Automatically get the local IP address of the PC
server_port = 8000
socketserver.bind((server_ip, server_port))
socketserver.listen(0)

print(f"Server is listening on {server_ip}:{server_port}")

# Accept a connection
connection = socketserver.accept()[0].makefile('rb')

try:
    while True:
        # Read the length of the image as a 32-bit unsigned int.
        image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
        if not image_len:
            break
        # Construct a stream to hold the image data and read the image data from the connection.
        image_stream = io.BytesIO()
        image_stream.write(connection.read(image_len))
        image_stream.seek(0)

        # Convert the image data to a numpy array.
        image_array = np.frombuffer(image_stream.read(), dtype=np.uint8)

        # Decode the image data using OpenCV.
        frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

        # Display the image using OpenCV.
        cv2.imshow('Video', frame)

        # Exit if 'q' is pressed.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    connection.close()
    socketserver.close()
    cv2.destroyAllWindows()