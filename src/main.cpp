#include <SPI.h>
#include <SimpleFOC.h>

class MotorFOC {
public:
  MotorFOC(int pwm_a, int pwm_b, int pwm_c, int en, int cs)
      : motor(7), driver(pwm_a, pwm_b, pwm_c, en), encoder(nullptr), cs(cs), last_print(0) {}

  void setup() {
    encoder = new MagneticSensorSPI(AS5048_SPI, cs);
    encoder->init();

    driver.voltage_power_supply = 10;
    driver.init();

    motor.linkSensor(encoder);
    motor.linkDriver(&driver);

    motor.controller = MotionControlType::angle;

    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 20;
    motor.voltage_limit = 10;
    motor.LPF_velocity.Tf = 0.02;
    motor.P_angle.P = 20;
    motor.velocity_limit = 4;

    motor.init();
    motor.initFOC();

    Serial.println("Motor ready.");
  }

  void commandMotor(Commander commander, char *cmd) { commander.motion(&motor, cmd); }

  void pollEncoder() {
    encoder->update();
    unsigned long now = millis();
    if (now - last_print >= 500) {
      last_print = now;
      Serial.println(encoder->getAngle());
    }
  }

private:
  BLDCMotor motor;
  BLDCDriver3PWM driver;
  MagneticSensorSPI *encoder;
  int cs;
  unsigned long last_print;
};

MotorFOC motor1(25, 26, 27, 33, 21);

Commander commander = Commander(Serial);

void onTarget(char *cmd) { motor1.commandMotor(commander, cmd); }

void setup() {
  Serial.begin(115200);
  delay(200);

  motor1.setup();
  commander.add('T', onTarget, "motion control");
}

void loop() {
  // motor.loopFOC();
  // motor.move();
  // command.run();

  motor1.pollEncoder();
  delay(50);
}