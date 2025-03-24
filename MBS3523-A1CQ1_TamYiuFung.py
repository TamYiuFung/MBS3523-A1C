import serial
import cv2
from threading import Thread

# Initialize serial communication
ser = serial.Serial('COM10', 9600, timeout=1)

# Initialize webcam
cap = cv2.VideoCapture(0)

# Global variables to store sensor data
temperature = "--"
humidity = "--"


# Function to read data from Arduino
def read_sensor_data():
    global temperature, humidity
    while True:
        if ser.in_waiting > 0:
            try:
                data = ser.readline().decode('utf-8').strip()
                if "," in data:  # Ensure valid data format
                    temp, humid = map(float, data.split(","))
                    temperature = f"{temp:.1f}"
                    humidity = f"{humid:.1f}"
            except:
                pass


# Function to stream webcam and overlay sensor data
def stream_webcam():
    while True:
        ret, frame = cap.read()
        if ret:
            # Overlay temperature and humidity text on the frame
            cv2.putText(frame, f"Temperature: {temperature} C",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, f"Humidity: {humidity} %",
                        (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Show the frame
            cv2.imshow("Webcam with DHT22 Data", frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    ser.close()


# Start sensor reading in a separate thread
sensor_thread = Thread(target=read_sensor_data)
sensor_thread.daemon = True
sensor_thread.start()

# Run webcam streaming in the main thread
stream_webcam()