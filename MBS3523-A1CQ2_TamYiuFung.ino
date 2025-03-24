#include <Servo.h>

Servo panServo;
Servo tiltServo;

const int PAN_PIN = 9;
const int TILT_PIN = 10;

int currentPanPos = 180;  // Updated initial position
int currentTiltPos = 45;   // Updated initial position

// Increased speed for tilt
const int MAX_SPEED_PAN = 3;
const int MAX_SPEED_TILT = 5;

void setup() {
  Serial.begin(9600);  // Match Python baudrate
  
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  
  // Set initial positions
  panServo.write(currentPanPos);
  tiltServo.write(currentTiltPos);
  
  delay(1000);
}

void moveServo(Servo &servo, int &currentPos, int change, bool isTilt) {
  int maxSpeed = isTilt ? MAX_SPEED_TILT : MAX_SPEED_PAN;
  
  // Limit the speed of movement
  change = constrain(change, -maxSpeed, maxSpeed);
  
  // Calculate new position
  int newPos = currentPos + change;
  
  // Constrain to valid range
  if (isTilt) {
    newPos = constrain(newPos, 0, 180);  // Tilt range: 0 to 180
  } else {
    newPos = constrain(newPos, 0, 180);  // Pan range: 0 to 180
  }
  
  // Update position and move servo
  if (newPos != currentPos) {
    currentPos = newPos;
    servo.write(currentPos);
  }
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    int commaIndex = command.indexOf(',');
    
    if (commaIndex != -1) {
      int panChange = command.substring(0, commaIndex).toInt();
      int tiltChange = command.substring(commaIndex + 1).toInt();
      
      moveServo(panServo, currentPanPos, panChange, false);
      moveServo(tiltServo, currentTiltPos, tiltChange, true);
    }
  }
  
  delay(10);
}