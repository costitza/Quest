#include <Servo.h>

// Motor pin definitions
const int motorPins[4][2] = {{A2, A3}, {A5, A4}, {9, 10}, {A0, A1}}; // Pins for four motors
//                    fata stanga, fata dreapta, spate stanga, spate dreapta
const int motorEnPins[2] = {3, 11}; // Enable pin for the motors

// Servo pin definitions
const int switchServosPins[4] = {2, 5, 4, 6};  // Pins for the drone mode servos
const int legServosPins[4] = {12, 7, 8, 13};    // Pins for the leg movement servos

int currentAngles[4] = {0, 0, 0, 0};

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

    Serial.begin(115200); // Match the baud rate used in Raspberry Pi code
    Serial.println("Arduino is ready to receive commands");
}

void driveWheels(int direction, int power) {
    // direction: 0=Stop, 1=Forward, 2=Backward, 3=Counterclockwise, 4=Clockwise
    switch(power) {
        case 4:
            for (int i = 0; i < 2; i++) {
                analogWrite(motorEnPins[i], 75);
            }
            break;
        case 0:
            for (int i = 0; i < 2; i++) {
                analogWrite(motorEnPins[i], 100);
            }
            break;
        case 1:
            for (int i = 0; i < 2; i++) {
                analogWrite(motorEnPins[i], 150);
            }
            break;
        case 2:
            for (int i = 0; i < 2; i++) {
                analogWrite(motorEnPins[i], 200);
            }
            break;
        case 3:
            for (int i = 0; i < 2; i++) {
                analogWrite(motorEnPins[i], 255);
            }
            break;
        case -1:
            for (int i = 0; i < 2; i++) {
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

void switchToDroneMode() {
    int currentAngles[4] = {switchServos[0].read(), switchServos[1].read(), switchServos[2].read(), switchServos[3].read()};
    int angles[4] = {115, 69, 177, 80};
    int anglesReached[4] = {0};

    while (anglesReached[0] + anglesReached[1] + anglesReached[2] + anglesReached[3] != 4) {
        for (int i = 0; i < 4; i++) {
            if (currentAngles[i] != angles[i]) {
                int increment = (currentAngles[i] < angles[i]) ? 1 : -1;
                currentAngles[i] += increment;
                switchServos[i].write(currentAngles[i]);
            } else {
                anglesReached[i] = 1;
            }
        }
        delay(5);
    }
}

void switchToDriveMode() {
    int currentAngles[4] = {switchServos[0].read(), switchServos[1].read(), switchServos[2].read(), switchServos[3].read()};
    int angles[4] = {16, 172, 73, 180};
    int anglesReached[4] = {0};

    while (anglesReached[0] + anglesReached[1] + anglesReached[2] + anglesReached[3] != 4) {
        for (int i = 0; i < 4; i++) {
            if (currentAngles[i] != angles[i]) {
                int increment = (currentAngles[i] < angles[i]) ? 1 : -1;
                currentAngles[i] += increment;
                switchServos[i].write(currentAngles[i]);
            } else {
                anglesReached[i] = 1;
            }
        }
        delay(5);
    }
}

void Drive(){
    int currentAnglesSwitch[4] = {switchServos[0].read(), switchServos[1].read(), switchServos[2].read(), switchServos[3].read()};
    int anglesSwitch[4] = {16, 172, 73, 180};
    int anglesSwitchReached[4] = {0};
    if(currentAnglesSwitch[0] != anglesSwitch[0]){
      int currentAnglesLegs[4] = {legServos[0].read(), legServos[1].read(), legServos[2].read(), legServos[3].read()};
      int anglesLegs[4] = {8, 157, 48, 160};
      int anglesLegsReached[4] = {0};
      while (anglesSwitchReached[0] + anglesSwitchReached[1] + anglesSwitchReached[2] + anglesSwitchReached[3] != 4 && 
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
    int anglesSwitch[4] = {115, 69, 177, 80};
    int anglesSwitchReached[4] = {0};
    if(currentAnglesSwitch[0] != anglesSwitch[0]){
      driveWheels(5, 0);
      adjustLegsStretched();
      driveWheels(0, -1);
      delay(300);
      int currentAnglesLegs[4] = {legServos[0].read(), legServos[1].read(), legServos[2].read(), legServos[3].read()};
      int anglesLegs[4] = {108, 52, 148, 60};
      int anglesLegsReached[4] = {0};
      while (anglesSwitchReached[0] + anglesSwitchReached[1] + anglesSwitchReached[2] + anglesSwitchReached[3] != 4 && 
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
    int angles[4] = {115, 52, 148, 60};
    int anglesReached[4] = {0};

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
        delay(10);
    }
}

void adjustLegsStretched() {
    int currentAngles[4] = {legServos[0].read(), legServos[1].read(), legServos[2].read(), legServos[3].read()};
    int angles[4] = {15, 152, 48, 160};
    int anglesReached[4] = {0};

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
        delay(10);
    }
}

void loop() {
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
