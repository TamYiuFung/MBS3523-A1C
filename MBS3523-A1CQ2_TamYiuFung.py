import cv2
import numpy as np
import serial
import time

# Create a named window for the trackbars
cv2.namedWindow('Trackbars', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Trackbars', 600, 600)

# Create a blank image (canvas)
canvas = np.zeros((600, 600, 3), dtype=np.uint8)

# Initialize serial communication
ser = serial.Serial('COM6', baudrate=9600, timeout=1)
time.sleep(2)

# Initialize video capture
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 30)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Add a flag to track the camera's position (upper or lower)
camera_upper_position = False  # Set to True if the camera is in the upper position

def nothing(x):
    pass

# Create trackbars for color detection
cv2.createTrackbar('HueLow', 'Trackbars', 11, 179, nothing)
cv2.createTrackbar('HueHigh', 'Trackbars', 104, 179, nothing)
cv2.createTrackbar('SatLow', 'Trackbars', 168, 255, nothing)
cv2.createTrackbar('SatHigh', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('ValLow', 'Trackbars', 139, 255, nothing)
cv2.createTrackbar('ValHigh', 'Trackbars', 255, 255, nothing)

# Create trackbars for PID control (reduced ranges)
cv2.createTrackbar('Kp Pan', 'Trackbars', 2, 20, nothing)  # Scale 0.0 to 0.2
cv2.createTrackbar('Ki Pan', 'Trackbars', 1, 20, nothing)  # Scale 0.0 to 0.02
cv2.createTrackbar('Kd Pan', 'Trackbars', 5, 20, nothing)  # Scale 0.0 to 0.2

cv2.createTrackbar('Kp Tilt', 'Trackbars', 2, 20, nothing)  # Scale 0.0 to 0.2
cv2.createTrackbar('Ki Tilt', 'Trackbars', 1, 20, nothing)  # Scale 0.0 to 0.02
cv2.createTrackbar('Kd Tilt', 'Trackbars', 5, 20, nothing)  # Scale 0.0 to 0.2

# Increased dead zone and reduced maximum movement
DEAD_ZONE_PAN = 30  # Increased dead zone
DEAD_ZONE_TILT = 30  # Increased dead zone
MAX_MOVEMENT_PAN = 5  # Reduced maximum movement
MAX_MOVEMENT_TILT = 5  # Reduced maximum movement

# PID variables
prev_error_x, prev_error_y = 0, 0
integral_x, integral_y = 0, 0

# Add variables for movement detection
last_positions = []
MAX_POSITIONS = 10
SPIN_THRESHOLD = 100  # Threshold for detecting spinning

def detect_spinning(positions, new_pos):
    positions.append(new_pos)
    if len(positions) > MAX_POSITIONS:
        positions.pop(0)

    if len(positions) < MAX_POSITIONS:
        return False

    # Calculate total movement
    total_movement = sum(abs(positions[i][0] - positions[i - 1][0]) for i in range(1, len(positions)))
    return total_movement > SPIN_THRESHOLD

def pid_control_pan(error, prev_error, integral):
    if abs(error) < DEAD_ZONE_PAN:
        integral = 0  # Reset integral when in dead zone
        return 0, integral

    # Reduced gains
    Kp_pan = cv2.getTrackbarPos('Kp Pan', 'Trackbars') / 100.0  # Scale down to 0.0-0.2
    Ki_pan = cv2.getTrackbarPos('Ki Pan', 'Trackbars') / 1000.0  # Scale down to 0.0-0.02
    Kd_pan = cv2.getTrackbarPos('Kd Pan', 'Trackbars') / 100.0  # Scale down to 0.0-0.2

    integral += error
    integral = max(-100, min(100, integral))  # Reduced integral limits

    derivative = error - prev_error
    output = Kp_pan * error + Ki_pan * integral + Kd_pan * derivative

    # Apply non-linear scaling to reduce sensitivity for small errors
    output = np.sign(output) * (abs(output) ** 0.7)

    output = max(-MAX_MOVEMENT_PAN, min(MAX_MOVEMENT_PAN, output))
    return output, integral

def pid_control_tilt(error, prev_error, integral):
    if abs(error) < DEAD_ZONE_TILT:
        integral = 0  # Reset integral when in dead zone
        return 0, integral

    # Reduced gains
    Kp_tilt = cv2.getTrackbarPos('Kp Tilt', 'Trackbars') / 100.0
    Ki_tilt = cv2.getTrackbarPos('Ki Tilt', 'Trackbars') / 1000.0
    Kd_tilt = cv2.getTrackbarPos('Kd Tilt', 'Trackbars') / 100.0

    integral += error
    integral = max(-100, min(100, integral))  # Reduced integral limits

    derivative = error - prev_error
    output = Kp_tilt * error + Ki_tilt * integral + Kd_tilt * derivative

    # Apply non-linear scaling
    output = np.sign(output) * (abs(output) ** 0.7)

    output = max(-MAX_MOVEMENT_TILT, min(MAX_MOVEMENT_TILT, output))
    return output, integral

# Increased smoothing
last_pan_output, last_tilt_output = 0, 0
smooth_factor = 0.85  # Increased smoothing factor

def smooth_output(new_value, last_value):
    return last_value * smooth_factor + new_value * (1 - smooth_factor)

def send_servo_command(pan, tilt):
    global last_pan_output, last_tilt_output

    # Invert tilt output if the camera is in the upper position
    if camera_upper_position:
        tilt = -tilt  # Invert the tilt direction

    # Additional smoothing for sudden changes
    if abs(pan - last_pan_output) > MAX_MOVEMENT_PAN:
        pan = last_pan_output + np.sign(pan - last_pan_output) * MAX_MOVEMENT_PAN

    if abs(tilt - last_tilt_output) > MAX_MOVEMENT_TILT:
        tilt = last_tilt_output + np.sign(tilt - last_tilt_output) * MAX_MOVEMENT_TILT

    smoothed_pan = smooth_output(pan, last_pan_output)
    smoothed_tilt = smooth_output(tilt, last_tilt_output)

    last_pan_output = smoothed_pan
    last_tilt_output = smoothed_tilt

    command = f"{int(smoothed_pan)},{int(smoothed_tilt)}\n"
    ser.write(command.encode())
    time.sleep(0.02)  # Increased delay

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get color threshold values
        hue_low = cv2.getTrackbarPos('HueLow', 'Trackbars')
        hue_high = cv2.getTrackbarPos('HueHigh', 'Trackbars')
        sat_low = cv2.getTrackbarPos('SatLow', 'Trackbars')
        sat_high = cv2.getTrackbarPos('SatHigh', 'Trackbars')
        val_low = cv2.getTrackbarPos('ValLow', 'Trackbars')
        val_high = cv2.getTrackbarPos('ValHigh', 'Trackbars')

        # Create mask
        lower_blue = np.array([hue_low, sat_low, val_low])
        upper_blue = np.array([hue_high, sat_high, val_high])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Enhanced noise reduction
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        mask = cv2.GaussianBlur(mask, (9, 9), 0)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw center crosshair
        center_x = frame.shape[1] // 2
        center_y = frame.shape[0] // 2
        cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)
        cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > 500:  # Minimum area threshold
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Check for spinning
                    if detect_spinning(last_positions, (cx, cy)):
                        # Reset PID controllers if spinning detected
                        integral_x = 0
                        integral_y = 0
                        continue

                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.drawContours(frame, [largest_contour], -1, (255, 0, 0), 2)

                    error_x = cx - frame.shape[1] // 2
                    error_y = cy - frame.shape[0] // 2

                    output_x, integral_x = pid_control_pan(error_x, prev_error_x, integral_x)
                    output_y, integral_y = pid_control_tilt(error_y, prev_error_y, integral_y)

                    prev_error_x = error_x
                    prev_error_y = error_y

                    send_servo_command(-output_x, output_y)

                    cv2.line(frame, (cx, cy), (center_x, center_y), (0, 255, 255), 2)

        cv2.imshow('Frame', frame)
        cv2.imshow('Mask', mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Program interrupted.")

finally:
    cap.release()
    cv2.destroyAllWindows()
    ser.close()
    print("Resources released.")