#include <Arduino.h>
#include <SimpleFOC.h>
#include <G431ESCEncoder.h>
#include <G431ESCDriver.h>
#include <HardwareTimer.h>

#define B_G431
#define ENCODER_PPR 1000

int currentcount = 0;
float target_angle = 0;

// Motor instance
BLDCMotor motor = BLDCMotor(7);

LowsideCurrentSense currentSense = LowsideCurrentSense(0.003, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

G431EscEncoder encoder = G431EscEncoder(ENCODER_PPR);

G431EscDriver6PWM driver = G431EscDriver6PWM(30000,75,12.0f,5.0f);


void printEncoder();

Commander commander = Commander(Serial);
void onMotor(char* cmd){ commander.motor(&motor,cmd); }
void doTarget(char* cmd){ commander.scalar(&target_angle,cmd); }





void setup() {
  HAL_Init();
  //Serial setup
  Serial.begin(2000000);
  Serial.println("STM32 B-G431B-ESC1 Tester");
  Serial.println("-------------------------");

  //Rotary encoder setup
  Serial.println("Starting encoder..");
  encoder.init();
  Serial.println("Encoder done!");

  motor.linkSensor(&encoder);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 2;
  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 4;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;
  // angle P controller
  motor.P_angle.P = 20;
  //  maximal velocity of the position control
  motor.velocity_limit = 4;

  //motor.useMonitoring(Serial);
  //motor.monitor_downsample = 0;
  // initialize motor

  // add target command T
  commander.add('M', onMotor, "motor");
  commander.add('T', doTarget, "target angle");

  commander.verbose = VerboseMode::user_friendly;

  motor.init();
  // align encoder and start FOC
  motor.initFOC();
  Serial.println(F("Motor ready."));
  _delay(500);
  Serial.println("Starting motion control!");
}

void loop(){
  motor.loopFOC();
  motor.move();
  motor.monitor();
  commander.run();
  //printEncoder();
  //delay(100);
}  




void printEncoder(){
  auto cnt = encoder.getEncoderCount();
  if(currentcount != cnt){
    currentcount = cnt;
    //printf("Count: %d\tTotal: %d\toverflow: %d\tangle: ", cnt,encoder.dbg_total(), encoder.dbg_overflow());

    Serial.print("Count: ");
    Serial.print(cnt);
    Serial.print("\tTotal: ");
    Serial.print(encoder.dbg_total());
    Serial.print("\tAngle: ");
    Serial.print(encoder.getAngle());
    Serial.print("\tdeg: ");
    Serial.println(encoder.getEncoderAngle());

  }
  
}

