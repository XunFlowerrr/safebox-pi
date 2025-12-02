import smbus
import math
import gpiod
import serial
import paho.mqtt.client as mqtt
import json
from time import sleep
from concurrent.futures import ThreadPoolExecutor
from flask import Flask, Response, render_template, redirect, url_for, request
import cv2
from deepface import DeepFace
import threading
import os
from picamera2 import Picamera2
import numpy as np

# --- MQTT Configuration ---
MQTT_BROKER = '10.146.134.87'  # Change to your MQTT broker IP
MQTT_PORT = 1883
MQTT_TOPICS = {
    'SENSOR_DATA': 'safebox/sensor-data',
    'SAFE_STATUS': 'safebox/safe-status',
    'ROTATION_DATA': 'safebox/rotation-data',
    'COMMAND': 'safebox/command',
}

# Initialize MQTT client
mqtt_client = mqtt.Client(client_id=f"safebox-gateway-{os.getpid()}")
mqtt_connected = False

def on_mqtt_connect(client, userdata, flags, rc):
    global mqtt_connected
    if rc == 0:
        print("Connected to MQTT broker")
        mqtt_connected = True
        # Subscribe to command topic to receive commands from backend
        client.subscribe(MQTT_TOPICS['COMMAND'])
    else:
        print(f"Failed to connect to MQTT broker, return code {rc}")
        mqtt_connected = False

def on_mqtt_disconnect(client, userdata, rc):
    global mqtt_connected
    mqtt_connected = False
    print("Disconnected from MQTT broker")

def on_mqtt_message(client, userdata, msg):
    """Handle incoming MQTT messages (commands from backend)"""
    global safe_status
    try:
        payload = json.loads(msg.payload.decode())
        print(f"Received command: {payload}")

        if msg.topic == MQTT_TOPICS['COMMAND']:
            command = payload.get('command')
            if command == 'unlock':
                safe_status = 'unlock'
                unlock()
            elif command == 'lock':
                safe_status = 'lock'
                lock()
    except Exception as e:
        print(f"Error processing MQTT message: {e}")

mqtt_client.on_connect = on_mqtt_connect
mqtt_client.on_disconnect = on_mqtt_disconnect
mqtt_client.on_message = on_mqtt_message

def connect_mqtt():
    """Connect to MQTT broker with retry logic"""
    global mqtt_connected
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
    except Exception as e:
        print(f"Error connecting to MQTT: {e}")
        mqtt_connected = False

# --- Flask App and Face Recognition ---
app = Flask(__name__)

# Camera setup
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

# Face recognition configuration
db_path = "photos"
if not os.path.exists(db_path):
    os.makedirs(db_path)

# Global variables
face_locations = []
face_names = []
is_processing = False
latest_frame = None
safe_status = 'lock'  # lock, unlock
allowed_users = ["mew"]

def check_face(frame):
    global face_locations, face_names, is_processing

    try:
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
        dfs = DeepFace.find(img_path=rgb_frame,
                            db_path=db_path,
                            model_name="VGG-Face",
                            enforce_detection=False,
                            silent=True)

        new_locations = []
        new_names = []
        authorized_person_detected = False

        if len(dfs) > 0:
            for df in dfs:
                if not df.empty:
                    x, y, w, h = df.iloc[0]['source_x'], df.iloc[0]['source_y'], df.iloc[0]['source_w'], df.iloc[0]['source_h']

                    full_path = df.iloc[0]['identity']
                    filename = os.path.basename(full_path)
                    name_part = os.path.splitext(filename)[0]
                    name = ''.join(filter(str.isalpha, name_part))

                    new_locations.append((x, y, w, h))
                    new_names.append(name.upper())

                    if name in allowed_users:
                        authorized_person_detected = True

        face_locations = new_locations
        face_names = new_names

        if authorized_person_detected:
            safe_status = 'unlock'

    except Exception as e:
        print(f"Error in AI thread: {e}")

    finally:
        is_processing = False

def generate_frames():
    global is_processing, face_locations, face_names, latest_frame
    while True:
        frame = picam2.capture_array()
        latest_frame = frame.copy()
        if not isinstance(frame, np.ndarray) or frame.size == 0:
            continue

        frame = cv2.flip(frame, 1)

        if not is_processing:
            is_processing = True
            threading.Thread(target=check_face, args=(frame.copy(),)).start()

        for (x, y, w, h), name in zip(face_locations, face_names):
            color = (0, 255, 0) if name.lower() in allowed_users else (0, 0, 255)
            cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
            cv2.rectangle(frame, (x, y-35), (x+w, y), color, cv2.FILLED)
            cv2.putText(frame, name, (x+6, y-6), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 1)

        if is_processing:
            cv2.circle(frame, (20, 20), 5, (0, 0, 255), -1)

        # Display safe status
        status_text = f"Safe: {safe_status.upper()}"
        status_color = (0, 255, 0) if safe_status == 'unlock' else (0, 0, 255)
        cv2.putText(frame, status_text, (frame.shape[1] - 150, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

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
    name = request.form.get('name')

    if latest_frame is not None and name:
        rgb_frame = cv2.cvtColor(latest_frame, cv2.COLOR_RGBA2RGB)
        rgb_frame = cv2.flip(rgb_frame, 1)

        i = 1
        while True:
            filename = f"{name}_{i}.jpg"
            filepath = os.path.join(db_path, filename)
            if not os.path.exists(filepath):
                break
            i += 1

        cv2.imwrite(filepath, rgb_frame)
        print(f"Image saved to {filepath}")

    return redirect(url_for('index'))

def run_flask_app():
    app.run(host='0.0.0.0', port=5000)

# --- Hardware Control ---

LED_PIN = 26
MAGNET_PIN = 16
BUZZER_PIN = 1

chip = gpiod.Chip('gpiochip0')
led_line = chip.get_line(LED_PIN)
led_line.request(consumer="LED", type=gpiod.LINE_REQ_DIR_OUT)
buzzer_line = chip.get_line(BUZZER_PIN)
buzzer_line.request(consumer="BUZZER", type=gpiod.LINE_REQ_DIR_OUT)
magnet_line = chip.get_line(MAGNET_PIN)
magnet_line.request(consumer="MAGNET", type=gpiod.LINE_REQ_DIR_IN)

ser = serial.Serial('/dev/ttyAMA0', 115200)

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

def MPU_Init():
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	bus.write_byte_data(Device_Address, CONFIG, 0)
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
        value = ((high << 8) | low)
        if(value > 32768):
                value = value - 65536
        return value

bus = smbus.SMBus(1)
Device_Address = 0x68

MPU_Init()

input_buffer = []
password = ['UP', 'DOWN', 'LEFT', 'RIGHT']
lock_cmd = ['DOWN', 'DOWN']

def lock():
    led_line.set_value(0)
    buzzer_line.set_value(1)
    sleep(0.5)
    buzzer_line.set_value(0)

def unlock():
    led_line.set_value(1)
    buzzer_line.set_value(1)
    sleep(0.1)
    buzzer_line.set_value(0)
    sleep(0.1)
    buzzer_line.set_value(1)
    sleep(0.1)
    buzzer_line.set_value(0)

def bad_input():
    buzzer_line.set_value(1)
    sleep(0.2)
    buzzer_line.set_value(0)
    sleep(0.2)
    buzzer_line.set_value(1)
    sleep(0.2)
    buzzer_line.set_value(0)
    sleep(0.2)
    buzzer_line.set_value(1)
    sleep(0.2)
    buzzer_line.set_value(0)

def beep():
    buzzer_line.set_value(1)
    sleep(0.1)
    buzzer_line.set_value(0)

def publish_data(data):
    """Publish sensor data to MQTT broker"""
    global mqtt_connected
    try:
        if not mqtt_connected:
            print("MQTT not connected, attempting to reconnect...")
            connect_mqtt()
            sleep(1)
            if not mqtt_connected:
                print("Failed to reconnect to MQTT")
                return

        print(f'Publishing data via MQTT: {data}')
        vibrate = data['vibrate']
        safeId = data['safeId']

        # Publish rotation data
        rotation_payload = json.dumps({
            'alpha': data['Gx'],
            'beta': data['Gy'],
            'gamma': data['Gz'],
            'safeId': safeId,
        })
        mqtt_client.publish(MQTT_TOPICS['ROTATION_DATA'], rotation_payload, qos=1)
        print(f"Published rotation data: {rotation_payload}")

        # Publish vibration data if available
        if len(vibrate) > 0:
            vibration_payload = json.dumps({
                'sensorType': 'vibration',
                'value': sum(vibrate) / len(vibrate),
                'safeId': safeId,
            })
            mqtt_client.publish(MQTT_TOPICS['SENSOR_DATA'], vibration_payload, qos=1)
            print(f"Published vibration data: {vibration_payload}")

        # Publish tilt data
        tilt_payload = json.dumps({
            'sensorType': 'tilt',
            'value': data['tilt'],
            'unit': '°/s',
            'safeId': safeId,
        })
        mqtt_client.publish(MQTT_TOPICS['SENSOR_DATA'], tilt_payload, qos=1)
        print(f"Published tilt data: {tilt_payload}")

        # Publish safe status
        status_payload = json.dumps({
            'status': data['status'],
            'safeId': safeId,
        })
        mqtt_client.publish(MQTT_TOPICS['SAFE_STATUS'], status_payload, qos=1)
        print(f"Published status: {status_payload}")

    except Exception as e:
        print(f'Error publishing to MQTT: {e}')

if __name__ == '__main__':
    # Connect to MQTT broker
    connect_mqtt()

    # Start Flask app in a new thread
    flask_thread = threading.Thread(target=run_flask_app)
    flask_thread.daemon = True
    flask_thread.start()

    with ThreadPoolExecutor() as executor:
        try:
            while True:
                try:
                    acc_x = read_raw_data(ACCEL_XOUT_H)
                    acc_y = read_raw_data(ACCEL_YOUT_H)
                    acc_z = read_raw_data(ACCEL_ZOUT_H)

                    gyro_x = read_raw_data(GYRO_XOUT_H)
                    gyro_y = read_raw_data(GYRO_YOUT_H)
                    gyro_z = read_raw_data(GYRO_ZOUT_H)

                    Ax = acc_x/16384.0
                    Ay = acc_y/16384.0
                    Az = acc_z/16384.0

                    Gx = gyro_x/131.0
                    Gy = gyro_y/131.0
                    Gz = gyro_z/131.0

                    print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)

                    tilt = math.sqrt(Gx**2 + Gy**2 + Gz**2)

                    magnet_value = magnet_line.get_value()
                    print(f'magnet = {magnet_value}')

                    if safe_status != 'unlock':
                        if magnet_value == 0: # closed
                            print('closed')
                            safe_satus = 'lock'
                            # buzzer_line.set_value(0)
                            led_line.set_value(0)
                        else: # open
                            print('open')
                            safe_status = 'open'
                            # buzzer_line.set_value(1)
                            led_line.set_value(1)

                    joystick = []
                    vibrate = []
                    try:
                        while ser.in_waiting > 0:
                            line = ser.readline().decode('utf-8').strip()
                            args = line.split()
                            try:
                                if args[0] == 'VIBRATED':
                                    vibrate.append(int(args[1]))
                                else:
                                    joystick.append(args[0])
                            except:
                                print('error(serial parser):', line)
                            print(f'serial: {line}')
                    except serial.SerialException as e:
                        print('error(serial)', e)

                    if len(joystick) > 0:
                        executor.submit(beep)
                        input_buffer.append(joystick[-1])
                        if safe_status == 'lock' or safe_status == 'open' and len(input_buffer) == len(password):
                            if input_buffer == password:
                                safe_status = 'unlock'
                                executor.submit(unlock)
                            else:
                                executor.submit(bad_input)
                            input_buffer = []
                        elif safe_status == 'unlock' and len(input_buffer) == len(lock_cmd):
                            if input_buffer == lock_cmd:
                                safe_status = 'lock'
                                executor.submit(lock)
                            else:
                                executor.submit(bad_input)
                            input_buffer = []

                    executor.submit(publish_data, {
                        'Gx': Gx,
                        'Gy': Gy,
                        'Gz': Gz,
                        'vibrate': vibrate,
                        'status': safe_status,
                        'tilt': tilt,
                        'safeId': 'safe-001',
                    })

                    sleep(1)
                except Exception as e:
                    print('error(main):', e)
        except Exception as e:
            print('error:', e)
        finally:
            # Cleanup
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
            buzzer_line.set_value(0)
            led_line.set_value(0)
            buzzer_line.release()
            led_line.release()
            magnet_line.release()
