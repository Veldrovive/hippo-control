// BLE velocity control: SimpleFOC closed-loop velocity control (same setup as
// constant_velocity_test) driven over Bluetooth LE. Pair with
// webapp/index.html (Web Bluetooth, Chrome/Edge).
//
// GATT service HIPPO_SERVICE_UUID:
//   CMD  (write)        ASCII command, same syntax as serial:
//                         V<rad/s>  target velocity     E1 / E0  enable / disable
//                         P<val>    vel loop P gain     I<val>   vel loop I gain
//                         F<sec>    vel LPF time const
//   TELEM (notify, 20B) little-endian: uint32 ms, float target_vel,
//                       float shaft_vel, float shaft_angle, float voltage_q
//   STATE (read/notify, 14B) little-endian: float P, float I, float F,
//                       uint8 motor_status, uint8 enabled
// 20-byte packets fit the default BLE MTU, so no MTU negotiation is needed.
//
// Safety: target velocity is set to 0 whenever the BLE client disconnects.
// Serial (115200) accepts the same commands and prints status lines.
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <SimpleFOC.h>
#include <Wire.h>

// PIN ASSIGNMENTS
#define I2C_SDA 21
#define I2C_SCL 22

#define MOTOR_IN1 14
#define MOTOR_IN2 25
#define MOTOR_IN3 26
#define MOTOR_EN 27

#define MOTOR_POLE_PAIRS 11
#define MOTOR_PHASE_RESISTANCE 7.2    // ohm
#define MOTOR_KV_RATING 61            // RPM / V
#define MOTOR_PHASE_INDUCTANCE 0.0026 // H

// HARDWARE CONFIG (see constant_velocity_test for the reasoning)
#define VOLTAGE_POWER_SUPPLY 24.0 // MUST match the real supply
#define DRIVER_VOLTAGE_LIMIT VOLTAGE_POWER_SUPPLY
#define MOTOR_VOLTAGE_LIMIT 12.0
#define CURRENT_LIMIT 1.0 // A
#define VELOCITY_LIMIT 1000.0
#define VOLTAGE_SENSOR_ALIGN 3.0
#define VOLTAGE_SENSOR_ALIGN_STEP 1.5
#define ALIGN_ATTEMPTS 3

// VELOCITY LOOP GAINS (output is current in A)
#define FOC_VEL_P 0.05
#define FOC_VEL_I 1.0
#define FOC_VEL_D 0.0
#define FOC_VEL_LPF_TF 0.03

#define TARGET_VELOCITY 0.0 // rad/s
#define TELEM_INTERVAL_MS 50  // 20 Hz to the web app
#define STATE_INTERVAL_MS 500 // gains/status refresh

// BLE
// Keep short: flags(3) + 128-bit UUID(18) + name must fit the 31-byte adv packet
#define BLE_DEVICE_NAME "Hippo"
#define HIPPO_SERVICE_UUID "6e1a0001-8b3c-4f2a-9d4e-2b7c1f0a5e01"
#define CMD_CHAR_UUID "6e1a0002-8b3c-4f2a-9d4e-2b7c1f0a5e01"
#define TELEM_CHAR_UUID "6e1a0003-8b3c-4f2a-9d4e-2b7c1f0a5e01"
#define STATE_CHAR_UUID "6e1a0004-8b3c-4f2a-9d4e-2b7c1f0a5e01"

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE,
                            MOTOR_KV_RATING, MOTOR_PHASE_INDUCTANCE);
BLDCDriver3PWM driver =
    BLDCDriver3PWM(MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_EN);

float target_velocity = TARGET_VELOCITY;
unsigned long last_telem_time = 0;
unsigned long last_state_time = 0;

BLEServer *ble_server = nullptr;
BLECharacteristic *telem_char = nullptr;
BLECharacteristic *state_char = nullptr;

// BLE callbacks run in the BLE task (other core), so they only hand data to
// loop() via these queues/flags; loop() is the only thing touching the motor.
#define CMD_MAX_LEN 32
struct CmdMsg {
  char text[CMD_MAX_LEN];
};
QueueHandle_t cmd_queue;
volatile bool ble_connected = false;
volatile bool ble_disconnect_event = false;
volatile bool ble_connect_event = false;

// ---------------------------------------------------------------- commands
void printSettings() {
  Serial.print("# V=");
  Serial.print(target_velocity, 3);
  Serial.print(" P=");
  Serial.print(motor.PID_velocity.P, 4);
  Serial.print(" I=");
  Serial.print(motor.PID_velocity.I, 4);
  Serial.print(" F=");
  Serial.print(motor.LPF_velocity.Tf, 4);
  Serial.print(" enabled=");
  Serial.println(motor.enabled);
}

void sendState(); // forward

void runCommand(const char *line) {
  while (*line == ' ')
    line++;
  if (*line == '\0')
    return;
  char cmd = line[0];
  float val = atof(line + 1);

  Serial.print("# rx \"");
  Serial.print(line);
  Serial.println("\"");

  switch (cmd) {
  case 'V':
  case 'v':
    target_velocity = constrain(val, -VELOCITY_LIMIT, VELOCITY_LIMIT);
    break;
  case 'P':
  case 'p':
    motor.PID_velocity.P = val;
    break;
  case 'I':
  case 'i':
    motor.PID_velocity.I = val;
    motor.PID_velocity.reset(); // clear integrator on gain change
    break;
  case 'F':
  case 'f':
    motor.LPF_velocity.Tf = val;
    break;
  case 'E':
  case 'e':
    if (val != 0 && motor.motor_status == FOCMotorStatus::motor_ready) {
      motor.PID_velocity.reset();
      motor.enable();
    } else {
      target_velocity = 0;
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

// Serial: non-blocking line reader (newline or 50 ms idle ends a command)
#define CMD_IDLE_MS 50
char serial_buf[CMD_MAX_LEN];
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
    } else if (serial_len < CMD_MAX_LEN - 1) {
      serial_buf[serial_len++] = c;
    }
  }
  if (serial_len > 0 && millis() - last_rx_time > CMD_IDLE_MS) {
    serial_buf[serial_len] = '\0';
    runCommand(serial_buf);
    serial_len = 0;
  }
}

// --------------------------------------------------------------------- BLE
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {
    ble_connected = true;
    ble_connect_event = true;
  }
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
    xQueueSend(cmd_queue, &msg, 0); // drop if full rather than block BLE task
  }
};

void sendTelemetry(unsigned long now) {
  uint8_t buf[20];
  uint32_t ms = now;
  float f[4] = {target_velocity, motor.shaft_velocity, motor.shaft_angle,
                motor.voltage.q};
  memcpy(buf, &ms, 4); // ESP32 is little-endian
  memcpy(buf + 4, f, 16);
  telem_char->setValue(buf, sizeof(buf));
  telem_char->notify();
}

void sendState() {
  if (!state_char)
    return;
  uint8_t buf[14];
  float f[3] = {motor.PID_velocity.P, motor.PID_velocity.I,
                motor.LPF_velocity.Tf};
  memcpy(buf, f, 12);
  buf[12] = (uint8_t)motor.motor_status;
  buf[13] = (uint8_t)motor.enabled;
  state_char->setValue(buf, sizeof(buf));
  if (ble_connected)
    state_char->notify();
}

void setupBLE() {
  cmd_queue = xQueueCreate(8, sizeof(CmdMsg));

  BLEDevice::init(BLE_DEVICE_NAME);
  ble_server = BLEDevice::createServer();
  ble_server->setCallbacks(new ServerCallbacks());

  BLEService *svc = ble_server->createService(HIPPO_SERVICE_UUID);

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
  // Bluedroid needs the CCCD added by hand; NimBLE adds it automatically.
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
  while (xQueueReceive(cmd_queue, &msg, 0) == pdTRUE) {
    runCommand(msg.text);
  }
  if (ble_connect_event) {
    ble_connect_event = false;
    Serial.println("# BLE connected");
  }
  if (ble_disconnect_event) {
    ble_disconnect_event = false;
    target_velocity = 0; // safety: stop if the controller goes away
    Serial.println("# BLE disconnected, target set to 0");
    delay(100); // let the stack settle before re-advertising
    BLEDevice::startAdvertising();
  }
}

// ------------------------------------------------------------- setup/loop
void setup() {
  Serial.begin(115200);
  delay(1000);
  SimpleFOCDebug::enable(&Serial);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  sensor.min_elapsed_time = 0.002; // 2 ms velocity window: less encoder noise
  sensor.init(&Wire);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.current_limit = CURRENT_LIMIT;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.velocity_limit = VELOCITY_LIMIT;
  motor.voltage_sensor_align = VOLTAGE_SENSOR_ALIGN;

  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::estimated_current;

  motor.PID_velocity.P = FOC_VEL_P;
  motor.PID_velocity.I = FOC_VEL_I;
  motor.PID_velocity.D = FOC_VEL_D;
  motor.LPF_velocity.Tf = FOC_VEL_LPF_TF;

  motor.init();
  for (int attempt = 1; attempt <= ALIGN_ATTEMPTS; attempt++) {
    Serial.print("# align attempt ");
    Serial.print(attempt);
    Serial.print(" at ");
    Serial.print(motor.voltage_sensor_align, 1);
    Serial.println(" V");
    if (motor.initFOC())
      break;
    motor.voltage_sensor_align += VOLTAGE_SENSOR_ALIGN_STEP;
    motor.enable(); // initFOC disables the motor on failure
    delay(200);
  }
  Serial.print("# motor_status=");
  Serial.println((int)motor.motor_status);

  // BLE after FOC init so alignment timing isn't disturbed by the radio
  setupBLE();
  sendState();
}

void loop() {
  motor.loopFOC();
  motor.move(target_velocity);

  handleSerial();
  handleBLE();

  unsigned long now = millis();
  if (ble_connected && now - last_telem_time >= TELEM_INTERVAL_MS) {
    last_telem_time = now;
    sendTelemetry(now);
  }
  if (now - last_state_time >= STATE_INTERVAL_MS) {
    last_state_time = now;
    sendState();
  }
}
