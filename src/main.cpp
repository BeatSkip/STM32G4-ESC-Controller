#include <Arduino.h>
#include <SimpleFOC.h>
#include <G431ESCEncoder.h>
#include <G431ESCDriver.h>
#include <HardwareTimer.h>


#define ENCODER_PPR 600

// Motor instance




G431ESCEncoder encoder = G431ESCEncoder(ENCODER_PPR);
G431ESCDriver6PWM driver = G431ESCDriver6PWM(50000,80,12.0f,12.0f);


HardwareTimer timenc(TIM4);


void printEncoder();
void overflow();

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
  timenc.attachInterrupt(overflow);
  Serial.println("Encoder done!");

}

void loop(){
  printEncoder();
  driver.setPwm(0,0,0);
  
  

  Serial.println("setting 0");
  delay(2500);
  driver.setPwm(7.5f,7.5f,7.5f);
  Serial.println("setting 15000");
  delay(2500);
}  




void printEncoder(){
  auto cnt = encoder.getEncoderCount();
  if(currentcount != cnt){
    currentcount = cnt;
    printf("Count: %d\tAngle: %f\r\n", cnt,encoder.getEncoderAngle());
  }
  
}

void overflow(){
   encoder.handleOverflow();
}

