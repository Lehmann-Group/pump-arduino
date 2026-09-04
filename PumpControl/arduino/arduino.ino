// Two 12 V brushed-DC peristaltic pumps controlled by two AOD4184 PWM MOSFET modules.
// Pump 0: Arduino D9  -> AOD4184 #0 PWM
// Pump 1: Arduino D10 -> AOD4184 #1 PWM
//
// Serial commands from Python:
//   a         Read both analog sensors: A,<sensor0>,<sensor1>
//   p0:0-255  Set Pump 0 PWM speed
//   p1:0-255  Set Pump 1 PWM speed
//   s         Report speeds: S,<pump0>,<pump1>
//   x         Stop both pumps

const byte NUM_PUMPS = 2;

const byte PUMP_PINS[NUM_PUMPS] = {
  9,   // Pump 0 PWM output
  10   // Pump 1 PWM output
};

const byte SENSOR_PINS[NUM_PUMPS] = {
  A0,  // Sensor 0
  A5   // Sensor 1
};

byte pumpSpeed[NUM_PUMPS] = {0, 0};

char commandBuffer[16];
byte commandIndex = 0;

void setup() {
  Serial.begin(9600);

  for (byte pump = 0; pump < NUM_PUMPS; pump++) {
    pinMode(PUMP_PINS[pump], OUTPUT);
    setPumpSpeed(pump, 0);  // Pumps off at startup
  }

  Serial.println("READY");
}

void loop() {
  while (Serial.available() > 0) {
    char incoming = Serial.read();

    // Python sends each command followed by "\n".
    if (incoming == '\n' || incoming == '\r') {
      if (commandIndex > 0) {
        commandBuffer[commandIndex] = '\0';
        processCommand(commandBuffer);
        commandIndex = 0;
      }
    }
    else if (commandIndex < sizeof(commandBuffer) - 1) {
      commandBuffer[commandIndex++] = incoming;
    }
    else {
      commandIndex = 0;
      Serial.println("ERR: command too long");
    }
  }
}

void processCommand(const char *command) {
  // Read analog sensors.
  if (strcmp(command, "a") == 0) {
    readSensors();
    return;
  }

  // Report current PWM commands.
  if (strcmp(command, "s") == 0) {
    reportPumpSpeeds();
    return;
  }

  // Emergency / normal stop: stop both pumps.
  if (strcmp(command, "x") == 0) {
    stopAllPumps();
    Serial.println("OK: stopped");
    return;
  }

  // Set speed commands:
  // p0:255 = Pump 0 full speed
  // p1:128 = Pump 1 approximately 50% PWM
  if (command[0] == 'p' &&
      (command[1] == '0' || command[1] == '1') &&
      command[2] == ':') {

    byte pump = command[1] - '0';
    long value = atol(command + 3);

    if (value < 0 || value > 255) {
      Serial.println("ERR: speed must be 0-255");
      return;
    }

    setPumpSpeed(pump, (byte)value);

    Serial.print("OK:p");
    Serial.print(pump);
    Serial.print(":");
    Serial.println(value);
    return;
  }

  Serial.println("ERR: unknown command");
}

void setPumpSpeed(byte pump, byte speedValue) {
  pumpSpeed[pump] = speedValue;
  analogWrite(PUMP_PINS[pump], speedValue);
}

void stopAllPumps() {
  for (byte pump = 0; pump < NUM_PUMPS; pump++) {
    setPumpSpeed(pump, 0);
  }
}

void readSensors() {
  int sensor0 = analogRead(SENSOR_PINS[0]);
  int sensor1 = analogRead(SENSOR_PINS[1]);

  Serial.print("A,");
  Serial.print(sensor0);
  Serial.print(",");
  Serial.println(sensor1);
}

void reportPumpSpeeds() {
  Serial.print("S,");
  Serial.print(pumpSpeed[0]);
  Serial.print(",");
  Serial.println(pumpSpeed[1]);
}