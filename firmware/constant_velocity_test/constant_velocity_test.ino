// Constant velocity test: spins the motor at a low fixed rate using SimpleFOC
// closed-loop velocity control. Hardware/pins/motor params match
// pid_teaching_tool.ino.
//
// Serial (115200), one command per line:
//   V<rad/s>  target velocity, e.g. "V2" or "V0"
//   P<val>    velocity loop P gain (A per rad/s)
//   I<val>    velocity loop I gain
//   F<sec>    velocity low-pass filter time constant
//   ?         print current settings
// Output is Serial Plotter format: target_vel, shaft_vel, sensor_vel (rad/s).
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

// HARDWARE CONFIG
// MUST match the real supply: PWM duty is computed from it, so a mismatch
// scales every commanded voltage (12 V here on a 24 V supply = 2x voltage).
#define VOLTAGE_POWER_SUPPLY 24.0
// Driver limit clamps each phase to 0..DRIVER_VOLTAGE_LIMIT, centered at half
// of it, so it must be ~the supply or it silently halves the usable voltage.
#define DRIVER_VOLTAGE_LIMIT VOLTAGE_POWER_SUPPLY
// Max phase voltage amplitude the motor may command. SVPWM tops out at
// Vsupply/sqrt(3) ~= 13.9 V on 24 V. This sets top speed (~KV * V).
#define MOTOR_VOLTAGE_LIMIT 12.0
// Caps the velocity loop's current command. With estimated_current the
// voltage is I*R + back-EMF, so this bounds stall/low-speed voltage to
// ~CURRENT_LIMIT * 7.2 ohm (heat), while back-EMF still lets V rise with speed.
#define CURRENT_LIMIT 1.0 // A
#define VELOCITY_LIMIT 1000.0
// Alignment voltage (absolute volts, independent of supply). 2 V (~0.28 A)
// was too weak to reliably move this motor; retries step it up.
#define VOLTAGE_SENSOR_ALIGN 3.0
#define VOLTAGE_SENSOR_ALIGN_STEP 1.5
#define ALIGN_ATTEMPTS 3

// VELOCITY LOOP GAINS (output is current in A; x7.2 ohm ~= volts)
// SimpleFOC defaults (P=0.5, I=10) oscillated on this gimbal motor.
#define FOC_VEL_P 0.05
#define FOC_VEL_I 1.0
#define FOC_VEL_D 0.0
#define FOC_VEL_LPF_TF 0.03

#define TARGET_VELOCITY 0.0 // rad/s
#define PLOT_INTERVAL_MS 200

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE,
                            MOTOR_KV_RATING, MOTOR_PHASE_INDUCTANCE);
BLDCDriver3PWM driver =
    BLDCDriver3PWM(MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_EN);

float target_velocity = TARGET_VELOCITY;
unsigned long last_plot_time = 0;

void printSettings() {
  Serial.print("# V=");
  Serial.print(target_velocity, 3);
  Serial.print(" P=");
  Serial.print(motor.PID_velocity.P, 4);
  Serial.print(" I=");
  Serial.print(motor.PID_velocity.I, 4);
  Serial.print(" F=");
  Serial.println(motor.LPF_velocity.Tf, 4);
}

// Non-blocking line reader. A command ends at '\n'/'\r', or after
// CMD_IDLE_MS of silence (so "No line ending" in the monitor also works).
#define CMD_IDLE_MS 50
char cmd_buf[32];
uint8_t cmd_len = 0;
unsigned long last_rx_time = 0;

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
  case '?':
    break;
  default:
    Serial.println("# unknown command");
    return;
  }
  printSettings();
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    last_rx_time = millis();
    if (c == '\n' || c == '\r') {
      cmd_buf[cmd_len] = '\0';
      runCommand(cmd_buf);
      cmd_len = 0;
    } else if (cmd_len < sizeof(cmd_buf) - 1) {
      cmd_buf[cmd_len++] = c;
    }
  }
  if (cmd_len > 0 && millis() - last_rx_time > CMD_IDLE_MS) {
    cmd_buf[cmd_len] = '\0';
    runCommand(cmd_buf);
    cmd_len = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // let the serial monitor attach so init messages aren't lost
  SimpleFOCDebug::enable(&Serial); // prints init/alignment progress

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // faster AS5600 reads -> faster FOC loop
  sensor.init(&Wire);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.current_limit = CURRENT_LIMIT;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // ~15% more V
  motor.velocity_limit = VELOCITY_LIMIT;
  motor.voltage_sensor_align = VOLTAGE_SENSOR_ALIGN;

  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::estimated_current;

  motor.PID_velocity.P = FOC_VEL_P;
  motor.PID_velocity.I = FOC_VEL_I;
  motor.PID_velocity.D = FOC_VEL_D;
  motor.LPF_velocity.Tf = FOC_VEL_LPF_TF;

  motor.init();
  // Sensor alignment: motor sweeps one electrical rev each way. Retry with
  // more voltage if it fails to notice movement.
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

  // motor_status: 4 = ready, 14 = calib failed, 15 = init failed (motor disabled)
  // sensor_direction: 1 = CW, -1 = CCW, 0 = UNKNOWN (alignment saw no motion)
  Serial.print("# motor_status=");
  Serial.print((int)motor.motor_status);
  Serial.print(" sensor_direction=");
  Serial.print((int)motor.sensor_direction);
  Serial.print(" zero_electric_angle=");
  Serial.println(motor.zero_electric_angle, 4);
}

void loop() {
  motor.loopFOC();
  motor.move(target_velocity);

  handleSerial();

  unsigned long now = millis();
  if (now - last_plot_time >= PLOT_INTERVAL_MS) {
    last_plot_time = now;
    Serial.print("target_vel:");
    Serial.print(target_velocity, 3);
    Serial.print(",shaft_vel:");
    Serial.print(motor.shaft_velocity, 3);
    Serial.print(",sensor_vel:"); // raw sensor, independent of FOC init
    Serial.println(sensor.getVelocity(), 3);
  }
}
