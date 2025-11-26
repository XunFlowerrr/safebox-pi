import cv2
from flask import Flask, Response, render_template, redirect, url_for
from deepface import DeepFace
import threading
import os
from picamera2 import Picamera2
import numpy as np
import time

app = Flask(__name__)

# Camera setup
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

# --- FACE RECOGNITION CONFIGURATION ---
db_path = "photos"
if not os.path.exists(db_path):
    os.makedirs(db_path)

# Global variables for face recognition
face_locations = []
face_names = []
is_processing = False  # Flag to check if AI is currently busy
latest_frame = None

# --- AI Function for Face Recognition (Runs in the background) ---
def check_face(frame):
    global face_locations, face_names, is_processing
    
    try:
        # Convert frame from RGBA to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)

        # Use a specific model for face recognition
        dfs = DeepFace.find(img_path=rgb_frame, 
                            db_path=db_path, 
                            model_name="VGG-Face", 
                            enforce_detection=False, 
                            silent=True)
        
        new_locations = []
        new_names = []

        if len(dfs) > 0:
            for df in dfs:
                if not df.empty:
                    # Get coordinates
                    x = int(df.iloc[0]['source_x'])
                    y = int(df.iloc[0]['source_y'])
                    w = int(df.iloc[0]['source_w'])
                    h = int(df.iloc[0]['source_h'])
                    
                    # Get name
                    full_path = df.iloc[0]['identity']
                    filename = os.path.basename(full_path)
                    name = os.path.splitext(filename)[0].upper()
                    
                    new_locations.append((x, y, w, h))
                    new_names.append(name)

        face_locations = new_locations
        face_names = new_names

    except Exception as e:
        print(f"Error in AI thread: {e}")
    
    finally:
        is_processing = False

# Frame generator for video stream
def generate_frames():
    global is_processing, face_locations, face_names, latest_frame
    while True:
        frame = picam2.capture_array()
        latest_frame = frame.copy()
        if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
            print("Ignoring empty camera frame.")
            continue

        frame = cv2.flip(frame, 1)
        
        # --- AI WORKER START ---
        if not is_processing:
            is_processing = True
            threading.Thread(target=check_face, args=(frame.copy(),)).start()

        # --- DRAWING ---
        if face_locations:
            for (x, y, w, h), name in zip(face_locations, face_names):
                # Draw Green Box
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.rectangle(frame, (x, y-35), (x+w, y), (0, 255, 0), cv2.FILLED)
                cv2.putText(frame, name, (x+6, y-6), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 1)
        
        if is_processing:
            cv2.circle(frame, (20, 20), 5, (0, 0, 255), -1) # Red dot for "Thinking"

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

@app.route('/capture', methods=['POST'])
def capture():
    global latest_frame
    if latest_frame is not None:
        # Convert to RGB before saving
        rgb_frame = cv2.cvtColor(latest_frame, cv2.COLOR_RGBA2RGB)
        rgb_frame = cv2.flip(rgb_frame, 1)
        
        # Create a unique filename
        person_name = "person" # You can change this to a name from a form input
        existing_files = len([name for name in os.listdir(db_path) if name.endswith(('.jpg', '.png'))])
        filename = f"{person_name}_{existing_files + 1}.jpg"
        filepath = os.path.join(db_path, filename)
        
        cv2.imwrite(filepath, rgb_frame)
        print(f"Image saved to {filepath}")

    return redirect(url_for('index'))

# Note: The '/battle' route and game.html might not function as expected
# as the hand gesture recognition part has been replaced.
@app.route('/battle', methods=['POST'])
def battle():
    # This part is from the original app and may need adjustments
    # as 'character' is no longer updated by hand gestures.
    return render_template('game.html', computer_choice="N/A", player_choice="N/A", result="Face Reco Mode")

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

