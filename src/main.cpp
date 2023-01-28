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
G431EscDriver6PWM driver = G431EscDriver6PWM(30000,75,12.0f,5.0f);

void printEncoder();

Commander commander = Commander(Serial);
void onMotor(char* cmd){ commander.motor(&motor,cmd); }


int currentcount = 0;

void setup() {
  HAL_Init();
  //Serial setup
  Serial.begin(2000000);
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

   // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set control loop type to be used
  motor.controller = MotionControlType::angle;

  // contoller configuration based on the controll type
  motor.P_angle.P = 0.01f;
  motor.P_angle.I = 0;
  motor.P_angle.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 3;

  // velocity low pass filtering time constant
  // angle loop velocity limit
  motor.velocity_limit = 5;


  motor.useMonitoring(Serial);
  //motor.monitor_downsample = 0;
  // initialize motor

  // add target command T
  commander.add('M', onMotor, "motor");
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
}  




void printEncoder(){
  auto cnt = encoder.getEncoderCount();
  if(currentcount != cnt){
    currentcount = cnt;
    printf("Count: %d\Total: %d\toverflow: %d\r\n", cnt,encoder.dbg_total(), encoder.dbg_overflow());
  }
  
}

