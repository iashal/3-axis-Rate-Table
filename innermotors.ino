#include <SimpleFOC.h>

InlineCurrentSense current_sense = InlineCurrentSense(0.01, 20, 39, _NC, 36);
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01, 20, 35, _NC, 34); // Replaced line

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C); // Replaced line
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32,33,25,22);

BLDCMotor motor1 = BLDCMotor(7); // Replaced line
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26,27,14,12); // Replaced line


float target_velocity = 0;
float target_velocity1 = 0; // Replaced line

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doTarget1(char* cmd) { command.scalar(&target_velocity1, cmd); } // Replaced line


void setup() {
  I2Cone.begin(19,18, 400000); 
  I2Ctwo.begin(23,5, 400000);
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo); // Replaced line
  
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1); // Replaced line

  driver.voltage_power_supply = 12;
  driver.init();
  current_sense.linkDriver(&driver);

  driver1.voltage_power_supply = 12; // Replaced line
  driver1.init(); // Replaced line
  current_sense1.linkDriver(&driver1); // Replaced line

  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1); // Replaced line
  
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM; // Replaced line
  
  motor.controller = MotionControlType::angle;
  motor1.controller = MotionControlType::angle; // Replaced line

  motor.PID_velocity.P = 4;
  motor1.PID_velocity.P = 0.3;
  motor.PID_velocity.I = 3;
  motor1.PID_velocity.I = 3;
  motor.PID_velocity.D = 0.0001;
  motor.P_angle.P = 5;
  motor1.P_angle.P = 15;



  motor.voltage_limit = 3;
  motor.voltage_sensor_align = 3;
  motor1.voltage_limit = 1.5;
  motor1.current_limit = 0.02;
  motor1.voltage_sensor_align = 3;

  motor.sensor_offset = -0.66;
  motor1.sensor_offset = 1.57;

  motor.LPF_velocity.Tf = 0.01;
  motor1.LPF_velocity.Tf = 0.02; // Replaced line

  motor.velocity_limit = 15;
  motor1.velocity_limit = 20; // Replaced line


  motor.motion_downsample = 5;
  motor1.motion_downsample = 5; // Replaced line

  motor.init();
  motor1.init(); // Replaced line

  Serial.begin(115200);
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial); // Replaced line
  motor.monitor_downsample = 1000;
  motor1.monitor_downsample = 1000; // Replaced line

  motor.init();
  motor1.init(); // Replaced line

  current_sense.init();
  current_sense1.init(); // Replaced line

  motor.linkCurrentSense(&current_sense);
  motor1.linkCurrentSense(&current_sense1); // Replaced line

  motor.initFOC(0.72404, Direction::CW);
  motor1.initFOC(2.94371, Direction::CW); // Replaced line

  command.add('P', doTarget, "target velocity");
  command.add('R', doTarget1, "target velocity"); // Replaced line

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
}

void loop() {
  Serial.print("P"); 
  Serial.print(sensor.getAngle()+0.66); 
  Serial.print(","); 
  Serial.print("R"); 
  Serial.print(sensor1.getAngle()-1.57); // Replaced line
  Serial.println();
  motor.loopFOC();
  motor1.loopFOC(); // Replaced line

  motor.move(target_velocity);
  motor1.move(target_velocity1); // Replaced line

  command.run();
}
