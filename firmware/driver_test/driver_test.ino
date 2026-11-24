// Driver test: checks the SimpleFOC Mini, motor wiring, and encoder one at a
// time, WITHOUT closed-loop FOC (no sensor alignment needed).
//
// Serial (115200), one command per line:
//   x         stop: all outputs off, driver disabled
//   h<V>      all three outputs at V (default half supply). No current flows.
//             Multimeter (DC) from each motor output to GND should read ~V.
//   a<V> b<V> c<V>
//             one phase at V, the other two at 0 V. Current flows, so the
//             motor should snap to and hold a position. a, b, c should each
//             hold a DIFFERENT position. Keep V small (default 3 V).
//   o<rad/s>  open-loop spin (no encoder used), e.g. "o5". "o0" holds still.
//             The encoder is read separately: its velocity should match the
//             commanded one. That checks encoder + pole pairs.
//   ?         print status
//
// Status prints every 500 ms: mode, encoder angle/velocity, I2C, fault pin.
// Hardware/pins/motor params match the other sketches.
#include <SimpleFOC.h>
#include <Wire.h>

// PIN ASSIGNMENTS
#define I2C_SDA 21
#define I2C_SCL 22

#define MOTOR_IN1 14
#define MOTOR_IN2 25
#define MOTOR_IN3 26
#define MOTOR_EN 27

// Optional: if your SimpleFOC Mini has a fault output (nFT / FAULT, open
// drain, LOW = fault), wire it to a free GPIO and set it here. -1 = unused.
#define FAULT_PIN -1

#define MOTOR_POLE_PAIRS 11
#define MOTOR_PHASE_RESISTANCE 7.2    // ohm
#define MOTOR_KV_RATING 61            // RPM / V
#define MOTOR_PHASE_INDUCTANCE 0.0026 // H

#define VOLTAGE_POWER_SUPPLY 24.0 // MUST match the real supply
#define PHASE_TEST_VOLTAGE 3.0    // default for a/b/c (~0.28 A through 10.8 ohm)
#define PHASE_TEST_MAX 6.0        // refuse anything above this for a/b/c
#define OPENLOOP_VOLTAGE 3.0      // voltage used for open-loop spin
#define STATUS_INTERVAL_MS 500

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE,
                            MOTOR_KV_RATING, MOTOR_PHASE_INDUCTANCE);
BLDCDriver3PWM driver =
    BLDCDriver3PWM(MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_EN);

enum Mode { MODE_OFF, MODE_RAW, MODE_OPENLOOP };
Mode mode = MODE_OFF;
float ua = 0, ub = 0, uc = 0; // raw phase voltages
float openloop_target = 0;    // rad/s
bool sensor_found = false;
unsigned long last_status = 0;

const char *modeName() {
  switch (mode) {
  case MODE_OFF: return "OFF";
  case MODE_RAW: return "RAW";
  case MODE_OPENLOOP: return "OPENLOOP";
  }
  return "?";
}

void stopAll() {
  mode = MODE_OFF;
  ua = ub = uc = 0;
  openloop_target = 0;
  driver.setPwm(0, 0, 0);
  driver.disable();
}

void setRaw(float a, float b, float c) {
  mode = MODE_RAW;
  ua = a; ub = b; uc = c;
  driver.enable();
  driver.setPwm(ua, ub, uc);
}

void printStatus() {
  Serial.print("# mode=");
  Serial.print(modeName());
  if (mode == MODE_RAW) {
    Serial.print(" Ua=");
    Serial.print(ua, 2);
    Serial.print(" Ub=");
    Serial.print(ub, 2);
    Serial.print(" Uc=");
    Serial.print(uc, 2);
  } else if (mode == MODE_OPENLOOP) {
    Serial.print(" cmd_vel=");
    Serial.print(openloop_target, 2);
  }
  Serial.print(" | enc_found=");
  Serial.print(sensor_found ? "yes" : "NO");
  Serial.print(" enc_angle_deg=");
  Serial.print(sensor.getAngle() * RAD_TO_DEG, 1);
  Serial.print(" enc_vel=");
  Serial.print(sensor.getVelocity(), 2);
  if (FAULT_PIN >= 0) {
    Serial.print(" | fault=");
    Serial.print(digitalRead(FAULT_PIN) == LOW ? "FAULT" : "ok");
  }
  Serial.println();
}

void runCommand(const char *line) {
  while (*line == ' ')
    line++;
  if (*line == '\0')
    return;
  char cmd = line[0];
  bool has_val = line[1] != '\0';
  float val = atof(line + 1);

  Serial.print("# rx \"");
  Serial.print(line);
  Serial.println("\"");

  switch (cmd) {
  case 'x':
  case 'X':
    stopAll();
    break;
  case 'h':
  case 'H': {
    float v = has_val ? val : VOLTAGE_POWER_SUPPLY / 2;
    v = constrain(v, 0, VOLTAGE_POWER_SUPPLY);
    setRaw(v, v, v);
    break;
  }
  case 'a': case 'A':
  case 'b': case 'B':
  case 'c': case 'C': {
    float v = has_val ? val : PHASE_TEST_VOLTAGE;
    if (v > PHASE_TEST_MAX) {
      Serial.print("# refusing: max for single-phase test is ");
      Serial.println(PHASE_TEST_MAX, 1);
      return;
    }
    v = max(v, 0.0f);
    char p = tolower(cmd);
    setRaw(p == 'a' ? v : 0, p == 'b' ? v : 0, p == 'c' ? v : 0);
    break;
  }
  case 'o':
  case 'O':
    mode = MODE_OPENLOOP;
    openloop_target = val;
    motor.enable();
    break;
  case '?':
    break;
  default:
    Serial.println("# unknown command");
    return;
  }
  printStatus();
}

// Non-blocking line reader (newline or 50 ms idle ends a command)
char serial_buf[32];
uint8_t serial_len = 0;
unsigned long last_rx_time = 0;

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    last_rx_time = millis();
    if (c == '\n' || c == '\r') {
      serial_buf[serial_len] = '\0';
      runCommand(serial_buf);
      serial_len = 0;
    } else if (serial_len < sizeof(serial_buf) - 1) {
      serial_buf[serial_len++] = c;
    }
  }
  if (serial_len > 0 && millis() - last_rx_time > 50) {
    serial_buf[serial_len] = '\0';
    runCommand(serial_buf);
    serial_len = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  SimpleFOCDebug::enable(&Serial);

  if (FAULT_PIN >= 0)
    pinMode(FAULT_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.beginTransmission(0x36);
  sensor_found = (Wire.endTransmission() == 0);
  sensor.init(&Wire);
  // Sensor deliberately NOT linked to the motor: open-loop ignores it, and
  // reading it separately lets us compare it against the commanded speed.

  driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
  driver.voltage_limit = VOLTAGE_POWER_SUPPLY;
  if (!driver.init())
    Serial.println("# ERROR: driver.init() failed (check pin assignments)");
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = OPENLOOP_VOLTAGE;
  motor.velocity_limit = 50;
  motor.init();

  stopAll();
  Serial.println("# driver_test ready. Commands: x, h<V>, a<V>, b<V>, c<V>, "
                 "o<rad/s>, ?");
  Serial.print("# supply setting: ");
  Serial.print(VOLTAGE_POWER_SUPPLY, 1);
  Serial.println(" V (must match the real supply)");
  printStatus();
}

void loop() {
  sensor.update();
  handleSerial();

  if (mode == MODE_OPENLOOP)
    motor.move(openloop_target);

  unsigned long now = millis();
  if (now - last_status >= STATUS_INTERVAL_MS) {
    last_status = now;
    printStatus();
  }
}
