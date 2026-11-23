// Encoder test: streams AS5600 angle for the Arduino Serial Plotter.
// Motor driver is held disabled so the shaft can be turned freely by hand.
// Hardware/pins match pid_teaching_tool.ino.
#include <SimpleFOC.h>
#include <Wire.h>

// PIN ASSIGNMENTS
#define I2C_SDA 21
#define I2C_SCL 22

#define MOTOR_EN 27

#define PLOT_INTERVAL_MS 200 // 50 Hz

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

unsigned long last_plot_time = 0;

void setup() {
  // Keep the driver off; we're only reading the sensor.
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, LOW);

  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  sensor.init(&Wire);

  // Quick I2C presence check for the AS5600 (address 0x36)
  Wire.beginTransmission(0x36);
  if (Wire.endTransmission() != 0) {
    Serial.println("# WARNING: AS5600 not found at 0x36 - check wiring/magnet");
  }
}

void loop() {
  sensor.update();

  unsigned long now = millis();
  if (now - last_plot_time >= PLOT_INTERVAL_MS) {
    last_plot_time = now;

    // Multi-turn shaft angle (deg) and single-turn mechanical angle (deg)
    float angle_deg = sensor.getAngle() * RAD_TO_DEG;
    float mech_deg = sensor.getMechanicalAngle() * RAD_TO_DEG;

    Serial.print("angle_deg:");
    Serial.print(angle_deg, 2);
    Serial.print(",mech_deg:");
    Serial.println(mech_deg, 2);
  }
}
