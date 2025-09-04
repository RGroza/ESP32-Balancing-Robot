#include <SPI.h>
#include <SimpleFOC.h>

class MotorFOC {
public:
  MotorFOC(int pwm_a, int pwm_b, int pwm_c, int en, int cs, const char *name)
      : motor_(7), driver_(pwm_a, pwm_b, pwm_c, en), encoder_(nullptr), cs_(cs), angle_(0), name_(name) {}

  void setup() {
    encoder_ = new MagneticSensorSPI(AS5048_SPI, cs_);
    encoder_->init();

    driver_.voltage_power_supply = 10;
    driver_.init();

    motor_.linkSensor(encoder_);
    motor_.linkDriver(&driver_);

    motor_.controller = MotionControlType::angle;

    motor_.PID_velocity.P = 0.2;
    motor_.PID_velocity.I = 20.0;
    motor_.voltage_limit = 4.0;
    motor_.LPF_velocity.Tf = 0.02;
    motor_.P_angle.P = 10.0;
    motor_.velocity_limit = 4.0;

    motor_.init();
    motor_.initFOC();

    Serial.printf("%s: zero_elec=%.3f dir=%s\n", name_, motor_.zero_electric_angle,
                  motor_.sensor_direction == Direction::CW ? "CW" : "CCW");
  }

  void run() {
    motor_.loopFOC();
    motor_.move();
  }

  void commandMotor(Commander commander, char *cmd) { commander.motion(&motor_, cmd); }

  void updateEncoder() {
    encoder_->update();
    angle_ = encoder_->getAngle();
  }

  float getAngle() { return angle_; }

  const char *getName() { return name_; }

private:
  BLDCMotor motor_;
  BLDCDriver3PWM driver_;
  MagneticSensorSPI *encoder_;
  int cs_;
  float angle_;
  const char *name_;
};

MotorFOC motor_left(27, 14, 12, 13, 2, "Left");
MotorFOC motor_right(32, 33, 25, 26, 4, "Right");

Commander commander = Commander(Serial);

unsigned long last_print = 0;
bool input_in_progress = false;

void onTarget(char *cmd) {
  motor_left.commandMotor(commander, cmd);
  motor_right.commandMotor(commander, cmd);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  motor_left.setup();
  motor_right.setup();

  commander.add('/', onTarget, "motion control");
}

void loop() {
  motor_left.run();
  motor_right.run();

  // motor_left.updateEncoder();
  // motor_right.updateEncoder();

  commander.run();
}