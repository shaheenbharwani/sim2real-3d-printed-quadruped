#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define FREQUENCY_OSCILLATOR 25000000
#define SERVO_FREQ 50
#define SERVO_MIN 102
#define SERVO_MAX 500

// Servos that should convert negative angles to positive angles
// Add the channels that aren't working properly here
int negativeAngleServos[] = {1, 5, 8, 10}; // front_left_knee, front_right_ankle, rear_right_ankle, rear_left_knee
int numNegativeAngleServos = 4;

void setup() {
  Serial.begin(9600);
  Serial.println("R3 with Basic Servo Control - Negative Angle Handler");
  
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(FREQUENCY_OSCILLATOR);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  
  Serial.println("Ready for servo commands");
  Serial.println("Commands: servo <channel> <angle>");
  Serial.println("Example: servo 0 90");
  
  // Print which servos handle negative angles
  Serial.print("Servos with negative angle conversion: ");
  for(int i = 0; i < numNegativeAngleServos; i++) {
    Serial.print(negativeAngleServos[i]);
    if(i < numNegativeAngleServos - 1) Serial.print(", ");
  }
  Serial.println();
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.startsWith("servo ")) {
      // Parse: "servo 0 90" or "servo 1 -35"
      int space1 = input.indexOf(' ');
      int space2 = input.indexOf(' ', space1 + 1);
      
      if (space1 > 0 && space2 > 0) {
        int channel = input.substring(space1 + 1, space2).toInt();
        int angle = input.substring(space2 + 1).toInt();
        
        setServoAngle(channel, angle);
        Serial.print("Moved servo ");
        Serial.print(channel);
        Serial.print(" to ");
        Serial.println(angle);
      }
    } else {
      Serial.print("Echo: ");
      Serial.println(input);
    }
  }
}

bool isNegativeAngleServo(int channel) {
  for(int i = 0; i < numNegativeAngleServos; i++) {
    if(negativeAngleServos[i] == channel) {
      return true;
    }
  }
  return false;
}

void setServoAngle(int channel, int angle) {
  int originalAngle = angle;
  
  // Handle negative angles for specific servos
  if (isNegativeAngleServo(channel) && angle < 0) {
    angle = abs(angle); // Convert negative to positive
    Serial.print("Channel ");
    Serial.print(channel);
    Serial.print(" converted ");
    Serial.print(originalAngle);
    Serial.print("° to ");
    Serial.print(angle);
    Serial.println("°");
  }
  
  // Constrain to servo range
  angle = constrain(angle, 0, 180);
  
  // Convert angle to PWM pulse length
  uint16_t pulseLength = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulseLength);
}