import pygame
import socket
import time
import threading
import cv2
import math
from queue import Queue

# Tello drone connection details
TELLO_IP = '192.168.10.1'
TELLO_PORT = 8889
LOCAL_PORT = 9000
VIDEO_PORT = 11111

# Initialize socket for drone communication
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', LOCAL_PORT))
sock.setblocking(False)  # Enable non-blocking for low-latency communication

# Load Haarcascade for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Command queue for reliable transmission
command_queue = Queue()
last_command = None  # To track the last sent command

# Drone behavior parameters
DESIRED_DISTANCE = 150
TOLERANCE = 10
HORIZONTAL_TOLERANCE = 30
HOVER_COMMAND = "rc 0 0 0 0"
frame_count = 0  # Counter for processing every nth frame

# Function to send commands to the drone
def send_command(command):
    global last_command
    if command != last_command:  # Only queue if the command is different
        command_queue.put(command)
        last_command = command

def process_command_queue():
    """Continuously process and send commands from the queue."""
    while True:
        if not command_queue.empty():
            command = command_queue.get()
            try:
                sock.sendto(command.encode('utf-8'), (TELLO_IP, TELLO_PORT))
                print(f"Command sent: {command}")
            except Exception as e:
                print(f"Failed to send command: {e}")
        time.sleep(0.03)  # Avoid overwhelming the network

# Initialize and return the joystick controller
def setup_controller():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise Exception("No joystick detected. Please connect a controller.")
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller initialized: {joystick.get_name()}")
    return joystick

# Handle joystick inputs and send commands to the drone
def process_joystick_input(joystick):
    pygame.event.pump()
    rc_values = {
        "lr": int(joystick.get_axis(0) * 100),
        "fb": int(-joystick.get_axis(1) * 100),
        "ud": int(-joystick.get_axis(3) * 100),
        "yaw": int(joystick.get_axis(2) * 100)
    }
    send_command(f"rc {rc_values['lr']} {rc_values['fb']} {rc_values['ud']} {rc_values['yaw']}")
    if joystick.get_button(0):
        send_command("takeoff")
    if joystick.get_button(1):
        send_command("land")

# Calculate Euclidean distance between two points
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Detect faces and adjust drone position
def detect_and_follow_face(frame):
    global frame_count
    frame_count += 1

    # Process every 2nd frame to reduce computational load
    if frame_count % 2 != 0:
        return False

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    if len(faces) > 0:
        # Use only the first detected face for efficiency
        x, y, w, h = faces[0]

        # Draw a rectangle around the detected face
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        frame_height, frame_width, _ = frame.shape
        face_center_x = x + w // 2
        face_center_y = y + h // 2
        current_distance = calculate_distance(face_center_x, face_center_y, frame_width // 2, frame_height // 2)
        horizontal_offset = face_center_x - (frame_width // 2)

        if horizontal_offset > HORIZONTAL_TOLERANCE:
            send_command("rc 0 0 0 20")
        elif horizontal_offset < -HORIZONTAL_TOLERANCE:
            send_command("rc 0 0 0 -20")
        else:
            if current_distance > DESIRED_DISTANCE + TOLERANCE:
                send_command("rc 0 20 0 0")
            elif current_distance < DESIRED_DISTANCE - TOLERANCE:
                send_command("rc 0 -20 0 0")
            else:
                send_command(HOVER_COMMAND)
        return True
    else:
        # No face detected, hover the drone
        send_command(HOVER_COMMAND)
        return False

# Stream video and handle face tracking
def start_video_stream():
    cap = cv2.VideoCapture(f'udp://{TELLO_IP}:{VIDEO_PORT}', cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("Video stream failed to open")
        return

    # Set lower resolution for faster processing
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)  # Ensure FPS matches Tello's capability
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    last_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Video frame not received")
            send_command(HOVER_COMMAND)
            continue

        current_time = time.time()
        fps = 1 / (current_time - last_time)
        print(f"FPS: {fps:.2f}")
        last_time = current_time

        # Process the frame for face detection and tracking
        detect_and_follow_face(frame)

        # Optionally display the video feed
        cv2.imshow('Tello Video Stream', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Main function to control the drone
def main():
    print("Initializing controller...")
    try:
        joystick = setup_controller()
    except Exception as e:
        print(e)
        return

    # Start command processing thread
    threading.Thread(target=process_command_queue, daemon=True).start()

    send_command("command")
    send_command("streamon")

    video_thread = threading.Thread(target=start_video_stream, daemon=True)
    video_thread.start()

    print("Controller ready. Press CTRL+C to stop.")
    try:
        while True:
            process_joystick_input(joystick)
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("Landing and exiting...")
        send_command("land")
    finally:
        send_command("streamoff")
        sock.close()
        pygame.quit()

if __name__ == "__main__":
    main()
