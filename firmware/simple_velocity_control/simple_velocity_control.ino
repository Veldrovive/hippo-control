// Simple velocity control: our own PI loop that outputs a VOLTAGE.
//
// SimpleFOC is only used for the low-level part (reading the encoder and
// turning "apply U volts of torque" into three phase voltages). Everything
// that decides how hard to push is in controlStep() below, so what you set is
// exactly what the motor gets: no current estimate, no hidden limits.
//
// Torque is proportional to phase current, and at standstill
// current = voltage / 7.2 ohm. So the voltage limit is the torque limit:
//   12 V -> ~1.7 A at stall (strong, but the motor heats up fast if stalled)
//    6 V -> ~0.8 A at stall
//
// Commands (serial 115200 or BLE, one per line):
//   V<rad/s>  velocity mode, target velocity      e.g. V10, V-5, V0
//   U<volts>  direct voltage mode (no loop): constant torque, e.g. U3
//   L<volts>  voltage (= torque) limit, max ~13.5 on 24 V
//   P<val>    Kp  (volts per rad/s of error)
//   I<val>    Ki  (volts per rad/s of error, per second)
//   F<sec>    velocity filter time constant
//   E1 / E0   enable / disable the motor
//   ?         print settings
//
// Works with webapp/index.html (same BLE protocol as ble_velocity_control).
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <SimpleFOC.h>
#include <Wire.h>

// ------------------------------------------------------------ hardware
#define I2C_SDA 21
#define I2C_SCL 22
#define MOTOR_IN1 14
#define MOTOR_IN2 25
#define MOTOR_IN3 26
#define MOTOR_EN 27
#define MOTOR_POLE_PAIRS 11

#define VOLTAGE_POWER_SUPPLY 24.0 // MUST match the real supply
#define VOLTAGE_SENSOR_ALIGN 3.0  // startup alignment push (volts)

// ------------------------------------------------------------ control
#define DEFAULT_VOLTAGE_LIMIT 12.0 // volts = torque limit (see top)
#define MAX_VOLTAGE_LIMIT 13.5     // ~ supply / sqrt(3) with space-vector PWM
#define DEFAULT_KP 0.3             // V per rad/s
#define DEFAULT_KI 3.0             // V per rad/s per s
#define DEFAULT_VEL_FILTER 0.03    // s
// Velocity = change in angle / time. The AS5600 has 4096 counts/rev
// (1.5 mrad each), so over SimpleFOC's default 0.1 ms window a single count
// of flicker reads as several rad/s of fake velocity. A 2 ms window cuts
// that ~20x for ~1 ms of extra lag.
#define VEL_SAMPLE_TIME 0.002 // s
#define CONTROL_PERIOD_US 1000     // run the PI loop at 1 kHz

enum Mode : uint8_t { MODE_VELOCITY = 0, MODE_VOLTAGE = 1 };

Mode mode = MODE_VELOCITY;
float target_velocity = 0; // rad/s  (velocity mode)
float direct_voltage = 0;  // V      (voltage mode)
float voltage_limit = DEFAULT_VOLTAGE_LIMIT;
float kp = DEFAULT_KP;
float ki = DEFAULT_KI;

float integral = 0;    // V, the I-term's accumulated output
float voltage_cmd = 0; // V, what we send to the motor

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver =
    BLDCDriver3PWM(MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_EN);

// =====================================================================
// THE CONTROL LOOP. Runs every CONTROL_PERIOD_US. dt is in seconds.
// =====================================================================
void controlStep(float dt) {
  float velocity = motor.shaft_velocity; // rad/s, already low-pass filtered

  if (mode == MODE_VELOCITY) {
    float error = target_velocity - velocity;

    // I term, clamped so it can't wind up past what the motor can use
    integral += ki * error * dt;
    integral = constrain(integral, -voltage_limit, voltage_limit);

    voltage_cmd = kp * error + integral;
  } else { // MODE_VOLTAGE: no loop, just push
    voltage_cmd = direct_voltage;
  }

  voltage_cmd = constrain(voltage_cmd, -voltage_limit, voltage_limit);
}
// =====================================================================

// ------------------------------------------------------------ BLE setup
#define BLE_DEVICE_NAME "Hippo" // short: must fit the 31-byte advert
#define HIPPO_SERVICE_UUID "6e1a0001-8b3c-4f2a-9d4e-2b7c1f0a5e01"
#define CMD_CHAR_UUID "6e1a0002-8b3c-4f2a-9d4e-2b7c1f0a5e01"
#define TELEM_CHAR_UUID "6e1a0003-8b3c-4f2a-9d4e-2b7c1f0a5e01"
#define STATE_CHAR_UUID "6e1a0004-8b3c-4f2a-9d4e-2b7c1f0a5e01"
#define TELEM_INTERVAL_MS 50
#define STATE_INTERVAL_MS 500

BLECharacteristic *telem_char = nullptr;
BLECharacteristic *state_char = nullptr;

// BLE callbacks run on the other core: they only queue commands; loop()
// is the only thing that touches the motor.
#define CMD_MAX_LEN 32
struct CmdMsg {
  char text[CMD_MAX_LEN];
};
QueueHandle_t cmd_queue;
volatile bool ble_connected = false;
volatile bool ble_disconnect_event = false;

unsigned long last_control_us = 0;
unsigned long last_telem_ms = 0;
unsigned long last_state_ms = 0;

// ------------------------------------------------------------ commands
void printSettings() {
  Serial.print("# mode=");
  Serial.print(mode == MODE_VELOCITY ? "velocity" : "voltage");
  Serial.print(" V=");
  Serial.print(target_velocity, 2);
  Serial.print(" U=");
  Serial.print(direct_voltage, 2);
  Serial.print(" L=");
  Serial.print(voltage_limit, 2);
  Serial.print(" P=");
  Serial.print(kp, 4);
  Serial.print(" I=");
  Serial.print(ki, 4);
  Serial.print(" F=");
  Serial.print(motor.LPF_velocity.Tf, 4);
  Serial.print(" enabled=");
  Serial.println(motor.enabled);
}

void sendState();

void runCommand(const char *line) {
  while (*line == ' ')
    line++;
  if (*line == '\0')
    return;
  char cmd = toupper(line[0]);
  float val = atof(line + 1);

  Serial.print("# rx \"");
  Serial.print(line);
  Serial.println("\"");

  switch (cmd) {
  case 'V':
    if (mode != MODE_VELOCITY)
      integral = 0; // fresh start when switching modes
    mode = MODE_VELOCITY;
    target_velocity = val;
    break;
  case 'U':
    mode = MODE_VOLTAGE;
    direct_voltage = val;
    break;
  case 'L':
    voltage_limit = constrain(val, 0.0f, (float)MAX_VOLTAGE_LIMIT);
    break;
  case 'P':
    kp = val;
    break;
  case 'I':
    ki = val;
    integral = 0;
    break;
  case 'F':
    motor.LPF_velocity.Tf = val;
    break;
  case 'E':
    if (val != 0 && motor.motor_status == FOCMotorStatus::motor_ready) {
      integral = 0;
      motor.enable();
    } else {
      target_velocity = 0;
      direct_voltage = 0;
      motor.disable();
    }
    break;
  case '?':
    break;
  default:
    Serial.println("# unknown command");
    return;
  }
  printSettings();
  sendState();
}

void stopMotion() {
  target_velocity = 0;
  direct_voltage = 0;
  integral = 0;
  mode = MODE_VELOCITY;
}

// Serial: non-blocking line reader (newline or 50 ms idle ends a command)
char serial_buf[CMD_MAX_LEN];
uint8_t serial_len = 0;
unsigned long last_rx_ms = 0;

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    last_rx_ms = millis();
    if (c == '\n' || c == '\r') {
      serial_buf[serial_len] = '\0';
      runCommand(serial_buf);
      serial_len = 0;
    } else if (serial_len < CMD_MAX_LEN - 1) {
      serial_buf[serial_len++] = c;
    }
  }
  if (serial_len > 0 && millis() - last_rx_ms > 50) {
    serial_buf[serial_len] = '\0';
    runCommand(serial_buf);
    serial_len = 0;
  }
}

// ------------------------------------------------------------ BLE
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override { ble_connected = true; }
  void onDisconnect(BLEServer *s) override {
    ble_connected = false;
    ble_disconnect_event = true;
  }
};

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue();
    CmdMsg msg;
    size_t n = min((size_t)v.length(), (size_t)(CMD_MAX_LEN - 1));
    memcpy(msg.text, v.c_str(), n);
    msg.text[n] = '\0';
    xQueueSend(cmd_queue, &msg, 0);
  }
};

// TELEM (20 B): uint32 ms, float target_vel, float velocity, float angle,
//               float voltage_cmd
void sendTelemetry(unsigned long now) {
  uint8_t buf[20];
  uint32_t ms = now;
  float f[4] = {mode == MODE_VELOCITY ? target_velocity : 0.0f,
                motor.shaft_velocity, motor.shaft_angle, voltage_cmd};
  memcpy(buf, &ms, 4);
  memcpy(buf + 4, f, 16);
  telem_char->setValue(buf, sizeof(buf));
  telem_char->notify();
}

// STATE (20 B): float P, float I, float F, uint8 motor_status,
//               uint8 enabled, uint8 mode, uint8 pad, float voltage_limit
// (first 14 bytes match ble_velocity_control)
void sendState() {
  if (!state_char)
    return;
  uint8_t buf[20] = {0};
  float f[3] = {kp, ki, motor.LPF_velocity.Tf};
  memcpy(buf, f, 12);
  buf[12] = (uint8_t)motor.motor_status;
  buf[13] = (uint8_t)motor.enabled;
  buf[14] = (uint8_t)mode;
  memcpy(buf + 16, &voltage_limit, 4);
  state_char->setValue(buf, sizeof(buf));
  if (ble_connected)
    state_char->notify();
}

void setupBLE() {
  cmd_queue = xQueueCreate(8, sizeof(CmdMsg));
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService *svc = server->createService(HIPPO_SERVICE_UUID);

  BLECharacteristic *cmd_char = svc->createCharacteristic(
      CMD_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE |
                         BLECharacteristic::PROPERTY_WRITE_NR);
  cmd_char->setCallbacks(new CmdCallbacks());
  telem_char = svc->createCharacteristic(TELEM_CHAR_UUID,
                                         BLECharacteristic::PROPERTY_NOTIFY);
  state_char = svc->createCharacteristic(
      STATE_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
#ifdef CONFIG_BLUEDROID_ENABLED
  telem_char->addDescriptor(new BLE2902());
  state_char->addDescriptor(new BLE2902());
#endif
  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(HIPPO_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("# BLE advertising as \"" BLE_DEVICE_NAME "\"");
}

void handleBLE() {
  CmdMsg msg;
  while (xQueueReceive(cmd_queue, &msg, 0) == pdTRUE)
    runCommand(msg.text);
  if (ble_disconnect_event) {
    ble_disconnect_event = false;
    stopMotion(); // safety: stop if the controller goes away
    Serial.println("# BLE disconnected, motion stopped");
    BLEDevice::startAdvertising();
  }
}

// ------------------------------------------------------------ setup/loop
void setup() {
  Serial.begin(115200);
  delay(1000);
  SimpleFOCDebug::enable(&Serial);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  sensor.min_elapsed_time = VEL_SAMPLE_TIME;
  sensor.init(&Wire);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
  driver.voltage_limit = VOLTAGE_POWER_SUPPLY;
  driver.init();
  motor.linkDriver(&driver);

  // Plain voltage torque mode: motor.move(U) applies exactly U volts
  // (clamped to motor.voltage_limit) along the torque axis.
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.voltage_limit = MAX_VOLTAGE_LIMIT; // our own limit is applied above
  motor.voltage_sensor_align = VOLTAGE_SENSOR_ALIGN;
  motor.LPF_velocity.Tf = DEFAULT_VEL_FILTER;

  motor.init();
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print("# align attempt ");
    Serial.print(attempt);
    Serial.print(" at ");
    Serial.print(motor.voltage_sensor_align, 1);
    Serial.println(" V");
    if (motor.initFOC())
      break;
    motor.voltage_sensor_align += 1.5;
    motor.enable(); // initFOC disables the motor on failure
    delay(200);
  }
  Serial.print("# motor_status=");
  Serial.print((int)motor.motor_status);
  Serial.println(" (4 = ready)");

  setupBLE();
  printSettings();
  sendState();
  last_control_us = micros();
}

void loop() {
  motor.loopFOC(); // reads encoder, applies the last voltage command

  unsigned long now_us = micros();
  if (now_us - last_control_us >= CONTROL_PERIOD_US) {
    float dt = (now_us - last_control_us) * 1e-6f;
    last_control_us = now_us;
    controlStep(dt);
  }
  motor.move(voltage_cmd); // also updates shaft_velocity / shaft_angle

  handleSerial();
  handleBLE();

  unsigned long now = millis();
  if (ble_connected && now - last_telem_ms >= TELEM_INTERVAL_MS) {
    last_telem_ms = now;
    sendTelemetry(now);
  }
  if (now - last_state_ms >= STATE_INTERVAL_MS) {
    last_state_ms = now;
    sendState();
  }
}
