#include <Servo.h>
#include <Arduino.h>
#include <NewPing.h>
#include <DHT.h>

// Ultrasonic definitions
const int FRONT_TRIG_PIN = 27;
const int FRONT_ECHO_PIN = 29;
const int RIGHT_TRIG_PIN = 23;
const int RIGHT_ECHO_PIN = 25;
const int LEFT_TRIG_PIN = 19;
const int LEFT_ECHO_PIN = 21;
const int BACK_TRIG_PIN = 15;
const int BACK_ECHO_PIN = 17;
const int MAX_DISTANCE = 200;

// Create NewPing instances for each sensor
NewPing front_sensor(FRONT_TRIG_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing right_sensor(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);
NewPing left_sensor(LEFT_TRIG_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing back_sensor(BACK_TRIG_PIN, BACK_ECHO_PIN, MAX_DISTANCE);

// Define pin for DHT sensor
const int DHT_PIN = 31;  // GPIO pin connected to DHT11 sensor

// Initialize DHT sensor
DHT dht(DHT_PIN, DHT11);

// Motor pin definitions
const int motorPins[4][2] = {{8, 10}, {14, 6}, {5, 7}, {9, 11}}; // Pins for four motors
//                    
const int motorEnPins[4] = {13, 12, 2, 3}; // Enable pin for the motors

// Servo pin definitions
const int switchServosPins[4] = {47, 44, 42, 40};  // Pins for the drone mode servos
const int legServosPins[4] = {38, 36, 34, 32};    // Pins for the leg movement servos

int currentAngles[4] = {0, 0, 0, 0};
// angle-uri pentru drive/drone mode la switch servo
int anglesDriveSwitch[4] = {20, 161, 71, 163};
int anglesDroneSwitch[4] = {119, 65, 169, 64};

// angle-uri pentru normal/streched la leg ervo
int anglesNormalLeg[4] = {151, 82, 83, 106};
int anglesStrechedLeg[4] = {71, 160, 3, 180};
int anglesNormalDroneLeg[4] = {151 , 85, 83, 108};

Servo switchServos[4];
Servo legServos[4];

void setup() {
    // Set up motor pins
    for (int i = 0; i < 4; i++) {
        pinMode(motorPins[i][0], OUTPUT);
        pinMode(motorPins[i][1], OUTPUT);
    }

    for (int i = 0; i < 2; i++) {
        pinMode(motorEnPins[i], OUTPUT);
    }

    // Attach switch servos
    for (int i = 0; i < 4; i++) {
        switchServos[i].attach(switchServosPins[i]);
    }

    // Attach leg servos
    for (int i = 0; i < 4; i++) {
        legServos[i].attach(legServosPins[i]);
    }

    for(int i=0; i < 4; i++)
    {
      switchServos[i].write(anglesDriveSwitch[i]);
    }

    for(int i = 0;i < 4; i++){
      legServos[i].write(anglesNormalLeg[i]);
    }

    // Initialize DHT sensor
    dht.begin();
    Serial.println("DHT sensor initialized.");

    Serial.begin(115200);
    Serial.println("Arduino is ready to receive commands");
}

void driveWheels(int direction, int power) {
    switch(power) {
        case 4:
            for (int i = 0; i < 4; i++) {
                analogWrite(motorEnPins[i], 75);
            }
            break;
        case 0:
            for (int i = 0; i < 4; i++) {
                analogWrite(motorEnPins[i], 100);
            }
            break;
        case 1:
            for (int i = 0; i < 4; i++) {
                analogWrite(motorEnPins[i], 150);
            }
            break;
        case 2:
            for (int i = 0; i < 4; i++) {
                analogWrite(motorEnPins[i], 200);
            }
            break;
        case 3:
            for (int i = 0; i < 4; i++) {
                analogWrite(motorEnPins[i], 255);
            }
            break;
        case -1:
            for (int i = 0; i < 4; i++) {
                analogWrite(motorEnPins[i], 0);
            }
            break;
    }

    switch (direction) {
        case 1:
            // Forward
            for (int i = 0; i < 4; i++) {
                digitalWrite(motorPins[i][0], HIGH);
                digitalWrite(motorPins[i][1], LOW);
            }
            break;
        case 2:
            // Backward
            for (int i = 0; i < 4; i++) {
                digitalWrite(motorPins[i][0], LOW);
                digitalWrite(motorPins[i][1], HIGH);
            }
            break;
        case 3:
            // Counterclockwise
            for (int i = 0; i < 3; i += 2) {
                digitalWrite(motorPins[i][0], LOW);
                digitalWrite(motorPins[i][1], HIGH);
            }
            for (int i = 1; i < 4; i += 2) {
                digitalWrite(motorPins[i][0], HIGH);
                digitalWrite(motorPins[i][1], LOW);
            }
            break;
        case 4:
            // Clockwise
            for (int i = 0; i < 3; i += 2) {
                digitalWrite(motorPins[i][0], HIGH);
                digitalWrite(motorPins[i][1], LOW);
            }
            for (int i = 1; i < 4; i += 2) {
                digitalWrite(motorPins[i][0], LOW);
                digitalWrite(motorPins[i][1], HIGH);
            }
            break;
        case 5:
            //Strech
            for (int i = 0; i < 2; i++) {
                digitalWrite(motorPins[i][0], HIGH);
                digitalWrite(motorPins[i][1], LOW);
            }
            for (int i = 2; i < 4; i++) {
                digitalWrite(motorPins[i][0], LOW);
                digitalWrite(motorPins[i][1], HIGH);
            }
            break;
        case 6:
            //Shrink
            for (int i = 0; i < 2; i++) {
                digitalWrite(motorPins[i][0], LOW);
                digitalWrite(motorPins[i][1], HIGH);
            }
            for (int i = 2; i < 4; i++) {
                digitalWrite(motorPins[i][0], HIGH);
                digitalWrite(motorPins[i][1], LOW);
            }
            break;
        case 0:
            for (int i = 0; i < 4; i++) {
                digitalWrite(motorPins[i][0], LOW);
                digitalWrite(motorPins[i][1], LOW);
            }
            break;
    }
}

void Drive(){
    int currentAnglesSwitch[4] = {switchServos[0].read(), switchServos[1].read(), switchServos[2].read(), switchServos[3].read()};
    int anglesSwitch[4];
    int anglesSwitchReached[4] = {0};
    
    for (int i = 0; i < 4; i++) {
        anglesSwitch[i] = anglesDriveSwitch[i];
    }

    if(currentAnglesSwitch[0] != anglesSwitch[0]) {
      int currentAnglesLegs[4] = {legServos[0].read(), legServos[1].read(), legServos[2].read(), legServos[3].read()};
      int anglesLegs[4];
      int anglesLegsReached[4] = {0};

      for (int i = 0; i < 4; i++) {
          anglesLegs[i] = anglesStrechedLeg[i];
      }

      while (anglesSwitchReached[0] + anglesSwitchReached[1] + anglesSwitchReached[2] + anglesSwitchReached[3] != 4 ||
      anglesLegsReached[0] + anglesLegsReached[1] + anglesLegsReached[2] + anglesLegsReached[3] != 4) {
          for (int i = 0; i < 4; i++) {
              if (currentAnglesSwitch[i] != anglesSwitch[i]) {
                  int increment = (currentAnglesSwitch[i] < anglesSwitch[i]) ? 1 : -1;
                  currentAnglesSwitch[i] += increment;
                  switchServos[i].write(currentAnglesSwitch[i]);
              } else {
                  anglesSwitchReached[i] = 1;
              }

              if (currentAnglesLegs[i] != anglesLegs[i]) {
                  int increment = (currentAnglesLegs[i] < anglesLegs[i]) ? 1 : -1;
                  currentAnglesLegs[i] += increment;
                  legServos[i].write(currentAnglesLegs[i]);
              } else {
                  anglesLegsReached[i] = 1;
              }
          }
          delay(10);
      }

      delay(300);
      driveWheels(6, 0);
      adjustLegsNormal();
      driveWheels(0, -1);
    }
}

void Drone(){
    int currentAnglesSwitch[4] = {switchServos[0].read(), switchServos[1].read(), switchServos[2].read(), switchServos[3].read()};
    int anglesSwitch[4];
    int anglesSwitchReached[4] = {0};

    for (int i = 0; i < 4; i++) {
        anglesSwitch[i] = anglesDroneSwitch[i];
    }

    if(currentAnglesSwitch[0] != anglesSwitch[0]){
      driveWheels(5, 0);
      adjustLegsStretched();
      driveWheels(0, -1);
      delay(300);

      int currentAnglesLegs[4] = {legServos[0].read(), legServos[1].read(), legServos[2].read(), legServos[3].read()};
      int anglesLegs[4]; // Declare the local array
      int anglesLegsReached[4] = {0};

      for (int i = 0; i < 4; i++) {
          anglesLegs[i] = anglesNormalDroneLeg[i];
      }

      while (anglesSwitchReached[0] + anglesSwitchReached[1] + anglesSwitchReached[2] + anglesSwitchReached[3] != 4 ||
      anglesLegsReached[0] + anglesLegsReached[1] + anglesLegsReached[2] + anglesLegsReached[3] != 4) {
          for (int i = 0; i < 4; i++) {
              if (currentAnglesSwitch[i] != anglesSwitch[i]) {
                  int increment = (currentAnglesSwitch[i] < anglesSwitch[i]) ? 1 : -1;
                  currentAnglesSwitch[i] += increment;
                  switchServos[i].write(currentAnglesSwitch[i]);
              } else {
                  anglesSwitchReached[i] = 1;
              }

              if (currentAnglesLegs[i] != anglesLegs[i]) {
                  int increment = (currentAnglesLegs[i] < anglesLegs[i]) ? 1 : -1;
                  currentAnglesLegs[i] += increment;
                  legServos[i].write(currentAnglesLegs[i]);
              } else {
                  anglesLegsReached[i] = 1;
              }
          }
          delay(10);
      }
    }
}

void adjustLegsNormal() {
    int currentAngles[4] = {legServos[0].read(), legServos[1].read(), legServos[2].read(), legServos[3].read()};
    int angles[4]; // Declare the local array
    int anglesReached[4] = {0};

    // Copy values from global anglesNormalLeg array
    for (int i = 0; i < 4; i++) {
        angles[i] = anglesNormalLeg[i];
    }

    while (anglesReached[0] + anglesReached[1] + anglesReached[2] + anglesReached[3] != 4) {
        for (int i = 0; i < 4; i++) {
            if (currentAngles[i] != angles[i]) {
                int increment = (currentAngles[i] < angles[i]) ? 1 : -1;
                currentAngles[i] += increment;
                legServos[i].write(currentAngles[i]);
            } else {
                anglesReached[i] = 1;
            }
        }
        delay(15);
    }
}

void adjustLegsStretched() {
    int currentAngles[4] = {legServos[0].read(), legServos[1].read(), legServos[2].read(), legServos[3].read()};
    int angles[4]; // Declare the local array
    int anglesReached[4] = {0};

    // Copy values from global anglesStrechedLeg array
    for (int i = 0; i < 4; i++) {
        angles[i] = anglesStrechedLeg[i];
    }

    while (anglesReached[0] + anglesReached[1] + anglesReached[2] + anglesReached[3] != 4) {
        for (int i = 0; i < 4; i++) {
            if (currentAngles[i] != angles[i]) {
                int increment = (currentAngles[i] < angles[i]) ? 1 : -1;
                currentAngles[i] += increment;
                legServos[i].write(currentAngles[i]);
            } else {
                anglesReached[i] = 1;
            }
        }
        delay(15);
    }
}

void loop() {
    // Read distances from ultrasonic sensors
  int front_distance = front_sensor.ping_cm();
  int right_distance = right_sensor.ping_cm();
  int left_distance = left_sensor.ping_cm();
  int back_distance = back_sensor.ping_cm();

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  Serial.print(front_distance);
  Serial.print(",");
  Serial.print(right_distance);
  Serial.print(",");
  Serial.print(left_distance);
  Serial.print(",");
  Serial.print(back_distance);
  Serial.print(",");
  Serial.print(temperature);
  Serial.print(",");
  Serial.println(humidity);
  if (Serial.available() > 0) {
      String incomingString = Serial.readStringUntil('\n');
      char mode = incomingString[0];
      if (mode == '0') {
          // Fly command
          Drone();
      } else if (mode == '1') {
          int direction = incomingString[1] - '0';
          int power = incomingString[2] - '0';
          // Drive command
          Drive();

          driveWheels(direction, power);
      }
  }
}
