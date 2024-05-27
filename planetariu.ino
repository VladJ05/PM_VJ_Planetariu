#include "BluetoothSerial.h"

#define NUM_MOTORS 8
#define NUM_HALL_SENSORS 8
BluetoothSerial SerialBT;

int motorsPins[NUM_MOTORS] = {13, 12, 14, 27, 26, 25, 33, 32};
int hallSensorsPins[NUM_HALL_SENSORS] = {35, 35, 35, 35, 35, 35, 35, 35};
int motorsState[NUM_MOTORS] = {1, 1, 1, 1, 1, 1, 1, 1};

int program_state = 0;
int hall_state = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialBT.begin("ESP32_Planetarium");
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(hallSensorsPins[i], INPUT);
    pinMode(motorsPins[i], OUTPUT);
    digitalWrite(motorsPins[i], HIGH);
  }
}

void move_one_step(int motor) {
  digitalWrite(motor, LOW);
  delay(50);
  digitalWrite(motor, HIGH);
}

void wait_but_read_hall(int time) {
  // for (int i = 0; i < time; i += 10) {
  //   if (!analogRead(hallSensorsPins[2]))
  //     hall_state = 1;
  //   delay(10);
  // }
  unsigned long startTime = millis();
  while (millis() - startTime < time) {
    int hallValue = analogRead(hallSensorsPins[2]);
    if (!hallValue) {
      hall_state = 1;
    }
}

void loop() {
  if (program_state == 0) {
    if (!analogRead(hallSensorsPins[2]))
      hall_state = 1;
    if (hall_state == 1) {
      // Here hard-coded because of lack of motors
      for (int i = 0; i < NUM_MOTORS; i++) {
        motorsState[i] = 0;
        hall_state = 0;
      }
    } else {
      move_one_step(motorsPins[2]);
      wait_but_read_hall(1000);
    }
    int change_state = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (!motorsState[i]) {
        change_state = 1;
        break;
      }
    }
    if (change_state) {
      program_state = 1;
      change_state = 0;
      digitalWrite(motorsPins[2], LOW);
      delay(100);
      digitalWrite(motorsPins[2], HIGH);
    }
  } else if (program_state == 1) {
    if (SerialBT.available()) {
      String message = SerialBT.readString();
      Serial.println("Received: " + message);

      if (message[0] == 'S' && message[1] == 'Y' && message[2] == 'N' && message[3] == 'C') {
        program_state = 0;
      } else if (message[0] == 'S' && message[1] == 'T' && message[2] == 'A' && message[3] == 'R' && message[4] == 'T') {
        program_state = 2;
      }
    }
  } else if (program_state == 2) {
    digitalWrite(motorsPins[2], LOW);
    delay(50);
    digitalWrite(motorsPins[2], HIGH);
    delay(200);
    if (SerialBT.available()) {
      String message = SerialBT.readString();
      Serial.println("Received: " + message);
      if (message[0] == 'S' && message[1] == 'T' && message[2] == 'O' && message[3] == 'P') {
        program_state = 1;
      }
    }
  }
}
