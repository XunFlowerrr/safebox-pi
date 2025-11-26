import cv2
from flask import Flask, Response, render_template
import mediapipe as mp
import random
from picamera2 import Picamera2
import numpy as np

app = Flask(__name__)

# Camera setup
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

# Finger landmark mapping
finger_landmarks = [
    ("Index_finger", mp.solutions.hands.HandLandmark.INDEX_FINGER_PIP, mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP),
    ("Middle_finger", mp.solutions.hands.HandLandmark.MIDDLE_FINGER_PIP, mp.solutions.hands.HandLandmark.MIDDLE_FINGER_TIP),
    ("Ring_finger", mp.solutions.hands.HandLandmark.RING_FINGER_PIP, mp.solutions.hands.HandLandmark.RING_FINGER_TIP),
    ("Pinky_finger", mp.solutions.hands.HandLandmark.PINKY_PIP, mp.solutions.hands.HandLandmark.PINKY_TIP),
]

finger_y_coordinates = {}
choice = ['PAPER', 'ROCK', 'SCISSORS']
character = "PAPER"  # default



# Hand classifier
def classifier(Index_finger_tip_y, Index_finger_pip_y, Middle_finger_tip_y, Middle_finger_pip_y, Ring_finger_tip_y, Ring_finger_pip_y, Pinky_finger_tip_y, Pinky_finger_pip_y):
    # TODO #1: Implement the `classifier` function to determine the gesture 
    # between "ROCK", "SCISSORS", or "PAPER" 
    index_open = Index_finger_tip_y < Index_finger_pip_y
    middle_open = Middle_finger_tip_y < Middle_finger_pip_y
    ring_open = Ring_finger_tip_y < Ring_finger_pip_y
    pinky_open = Pinky_finger_tip_y < Pinky_finger_pip_y

    if index_open and middle_open and ring_open and pinky_open:
        return "PAPER"
    elif index_open and middle_open and not ring_open and not pinky_open:
        return "SCISSOR"
    elif not index_open and not middle_open and not ring_open and not pinky_open:
        return "ROCK"
    elif index_open and pinky_open and not middle_open and not ring_open:
        return "LOVE"
    else:
        return ""

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Frame generator
def generate_frames():
    global character
    with mp_hands.Hands(
        model_complexity=0,
        min_detection_confidence=0.3,
        min_tracking_confidence=0.5,
        max_num_hands=2) as hands:

        while True:
            frame = picam2.capture_array()
            if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
                print("Ignoring empty camera frame.")
                continue

            frame = cv2.flip(frame, 1)

            # Process with MediaPipe
            frame.flags.writeable = False
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(rgb_frame)

            frame.flags.writeable = True
            frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
            frame_height, frame_width, _ = frame.shape

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    x_, y_ = [], []
                    for lm in hand_landmarks.landmark:
                        x_.append(lm.x)
                        y_.append(lm.y)

                    # Save finger y coordinates
                    for finger_name, pip_landmark, tip_landmark in finger_landmarks:
                        finger_y_coordinates[f"{finger_name}_pip_y"] = hand_landmarks.landmark[pip_landmark].y * frame_height
                        finger_y_coordinates[f"{finger_name}_tip_y"] = hand_landmarks.landmark[tip_landmark].y * frame_height

                    # Unpack variables safely
                    Index_finger_tip_y = finger_y_coordinates["Index_finger_tip_y"]
                    Index_finger_pip_y = finger_y_coordinates["Index_finger_pip_y"]
                    Middle_finger_tip_y = finger_y_coordinates["Middle_finger_tip_y"]
                    Middle_finger_pip_y = finger_y_coordinates["Middle_finger_pip_y"]
                    Ring_finger_tip_y = finger_y_coordinates["Ring_finger_tip_y"]
                    Ring_finger_pip_y = finger_y_coordinates["Ring_finger_pip_y"]
                    Pinky_finger_tip_y = finger_y_coordinates["Pinky_finger_tip_y"]
                    Pinky_finger_pip_y = finger_y_coordinates["Pinky_finger_pip_y"]

                    # Classify gesture
                    character = classifier(Index_finger_tip_y, Index_finger_pip_y, Middle_finger_tip_y, Middle_finger_pip_y, Ring_finger_tip_y, Ring_finger_pip_y, Pinky_finger_tip_y, Pinky_finger_pip_y)

                    # Draw hand landmarks
                    mp_drawing.draw_landmarks(
                        frame,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style())

                    # Draw bounding box & text
                    x1 = int(min(x_) * frame_width) - 10
                    y1 = int(min(y_) * frame_height) - 10
                    x2 = int(max(x_) * frame_width) + 10
                    y2 = int(max(y_) * frame_height) + 10

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 4)
                    cv2.putText(frame, character, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 0), 3, cv2.LINE_AA)

            # Encode to JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/battle', methods=['POST'])
def battle():
    choices = ["ROCK", "PAPER", "SCISSOR"]
    computer_choice = random.choice(choices)
    player_choice = character
    # result = "You Win"
    if player_choice == computer_choice:
        result = "Tie"
    elif (player_choice == "ROCK" and computer_choice == "SCISSOR") or (player_choice == "PAPER" and computer_choice == "ROCK") or (player_choice == "SCISSOR" and computer_choice == "PAPER"):
        result = "You Win"
    elif (player_choice == "LOVE"):
        result = "LOVE YOU 💓"
    else:
        result = "You Lose"
    return render_template('game.html', computer_choice=computer_choice, player_choice=player_choice, result=result)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

