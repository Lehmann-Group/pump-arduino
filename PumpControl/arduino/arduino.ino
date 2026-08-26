// Library imports
#include <Servo.h>

// Number of independently controlled pumps
const byte NUM_PUMPS = 2;

// Servo/ESC objects: waterPump[0] controls pump 0,
// waterPump[1] controls pump 1.
Servo waterPump[NUM_PUMPS];

// Pin declarations
const byte PUMP_PINS[NUM_PUMPS] = {
  9,   // Pump 0 signal pin
  10   // Pump 1 signal pin
};

// Analog sensor input pins
const byte SENSOR_PINS[NUM_PUMPS] = {
  A0,  // Sensor 0
  A5   // Sensor 1
};

// Sensor readings
int data[NUM_PUMPS] = {0, 0};

// Continuous-rotation servo / ESC command values
const int PUMP_ON = 180;
const int PUMP_OFF = 90;


void setup() {
  Serial.begin(9600);

  // Attach both pumps using one loop
  for (byte pump = 0; pump < NUM_PUMPS; pump++) {
    waterPump[pump].attach(PUMP_PINS[pump]);
    stopPump(pump);  // Ensure pumps are stopped at startup
  }

}


void loop() {
  if (Serial.available() > 0) {
    char userInput = Serial.read();

    switch (userInput) {
      case 'a':
        readAnalog();
        break;

      // Pump 0
      case 'c':
        runPump(0);
        break;

      case 'd':
        stopPump(0);
        break;

      // Pump 1
      case 'f':
        runPump(1);
        break;

      case 'g':
        stopPump(1);
        break;
    }
  }
}


// Read both analog sensors
void readAnalog() {
  for (byte sensor = 0; sensor < NUM_PUMPS; sensor++) {
    data[sensor] = analogRead(SENSOR_PINS[sensor]);
  }

  Serial.print(data[0]);
  Serial.print(",");
  Serial.println(data[1]);
}


// Start one selected pump
void runPump(byte pump) {
  if (pump >= NUM_PUMPS) return;
  waterPump[pump].write(PUMP_ON);
}

void stopPump(byte pump) {
  if (pump >= NUM_PUMPS) return;
  waterPump[pump].write(PUMP_OFF);
}