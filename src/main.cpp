#include <Arduino.h>
#include <SimpleFOC.h>
#include <G431ESCEncoder.h>
#include <G431ESCDriver.h>
#include <HardwareTimer.h>


#define ENCODER_PPR 600

// Motor instance
BLDCMotor motor = BLDCMotor(7);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
G431EscEncoder encoder = G431EscEncoder(ENCODER_PPR);
G431EscDriver6PWM driver = G431EscDriver6PWM(50000,80,12.0f,12.0f);

void printEncoder();

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.motion(&motor, cmd); }

int currentcount = 0;

void setup() {
  HAL_Init();
  //Serial setup
  Serial.begin(1000000);
  Serial.println("STM32 B-G431B-ESC1 Tester");
  Serial.println("-------------------------");

  //Timer Setup
  driver.init();

  //Rotary encoder setup
  Serial.println("Starting encoder..");
  encoder.init();
  Serial.println("Encoder done!");

  //SimpleFOC setup
  motor.linkSensor(&encoder);
  motor.linkDriver(&driver);
  currentSense.linkDriver(&driver);
  // current sensing
  currentSense.init();
  // no need for aligning
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

  // contoller configuration 
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
 
  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;

  // angle P controller
  motor.P_angle.P = 20;
  //  maximal velocity of the position control
  motor.velocity_limit = 4;

  motor.useMonitoring(Serial);
  
  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target angle");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
  motor.enable();
}

void loop(){
  motor.move();
  command.run();
}  




void printEncoder(){
  auto cnt = encoder.getEncoderCount();
  if(currentcount != cnt){
    currentcount = cnt;
    printf("Count: %d\tAngle: %f\r\n", cnt,encoder.getEncoderAngle());
  }
  
}

