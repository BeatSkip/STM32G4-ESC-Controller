#include "G431ESCDriver.h"


G431ESCDriver6PWM::G431ESCDriver6PWM(uint32_t pwmfreq, uint32_t dead_time, float psu, float limit){
  // Pin initialization
  

  // default power-supply value
  voltage_power_supply = psu;
  voltage_limit = limit;
  pwm_frequency = pwmfreq;

  // dead zone initial - 2%
  dead_zone = dead_time;

}

// enable motor driver
void  G431ESCDriver6PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( _isset(enable_pin) ) digitalWrite(enable_pin, enable_active_high);
    // set zero to PWM
    setPwm(0, 0, 0);
}

// disable motor driver
void G431ESCDriver6PWM::disable()
{
  // set zero to PWM
  setPwm(0, 0, 0);
  // disable the driver - if enable_pin pin available
  if ( _isset(enable_pin) ) digitalWrite(enable_pin, !enable_active_high);

}

// init hardware pins
int G431ESCDriver6PWM::init() {

    MyTim1 = pwm_init_high_and_low(IN1_H, IN1_L, IN2_H, IN2_L, IN3_H, IN3_L, pwm_frequency);
    MyTim1->resume();
    if(!MyTim1->isRunning()){
      Serial.println("ERROR!");
    }

    // sanity check for the voltage limit configuration
    if( !_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

    initialized = (params!=SIMPLEFOC_DRIVER_INIT_FAILED);
    return params!=SIMPLEFOC_DRIVER_INIT_FAILED;
}


// Set voltage to the pwm pin
void G431ESCDriver6PWM::setPwm(float Ua, float Ub, float Uc) {
  // limit the voltage in driver
  Ua = _constrain(Ua, 0, voltage_limit);
  Ub = _constrain(Ub, 0, voltage_limit);
  Uc = _constrain(Uc, 0, voltage_limit);
  // calculate duty cycle
  // limited in [0,1]
  dc_a = (Ua / voltage_power_supply * 8192);
  dc_b = (Ub / voltage_power_supply * 8192);
  dc_c = (Uc / voltage_power_supply * 8192);
  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  //MyTim1->setCaptureCompare(1, (int)dc_a, RESOLUTION_13B_COMPARE_FORMAT);
  //MyTim1->setCaptureCompare(2, (int)dc_b, RESOLUTION_13B_COMPARE_FORMAT);
  //MyTim1->setCaptureCompare(3, (int)dc_c, RESOLUTION_13B_COMPARE_FORMAT);
  pwm_manual(dc_a,dc_b,dc_c);
}

void G431ESCDriver6PWM::pwm_manual(uint16_t Ua, uint16_t Ub, uint16_t Uc) {
  pwm_set(A_PHASE_UH,Ua, RESOLUTION_13B_COMPARE_FORMAT);
  pwm_set(A_PHASE_VH,Ub, RESOLUTION_13B_COMPARE_FORMAT);
  pwm_set(A_PHASE_WH,Uc, RESOLUTION_13B_COMPARE_FORMAT);
}


HardwareTimer* G431ESCDriver6PWM::pwm_init_high_and_low(int uhPin, int ulPin, int vhPin, int vlPin, int whPin, int wlPin,uint32_t PWM_freq)
{
  PinName uhPinName = digitalPinToPinName(uhPin);
  PinName ulPinName = digitalPinToPinName(ulPin);
  PinName vhPinName = digitalPinToPinName(vhPin);
  PinName vlPinName = digitalPinToPinName(vlPin);
  PinName whPinName = digitalPinToPinName(whPin);
  PinName wlPinName = digitalPinToPinName(wlPin);

  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(uhPinName, PinMap_PWM);
 
  uint32_t index = get_timer_index(Instance);
  Serial.println("timer: "); Serial.println(index);


  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(uhPinName, PinMap_PWM));
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
    ((HardwareTimer *)HardwareTimer_Handle[index]->__this)->setOverflow(PWM_freq, HERTZ_FORMAT);
  }
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  uint32_t channelU = STM_PIN_CHANNEL(pinmap_function(uhPinName, PinMap_PWM));
  uint32_t channelV = STM_PIN_CHANNEL(pinmap_function(vhPinName, PinMap_PWM));
  uint32_t channelW = STM_PIN_CHANNEL(pinmap_function(whPinName, PinMap_PWM));

  
  HT->setMode(channelU, TIMER_OUTPUT_COMPARE_PWM1, uhPinName);
  HT->setMode(channelU, TIMER_OUTPUT_COMPARE_PWM1, ulPinName);
  HT->setMode(channelV, TIMER_OUTPUT_COMPARE_PWM1, vhPinName);
  HT->setMode(channelV, TIMER_OUTPUT_COMPARE_PWM1, vlPinName);
  HT->setMode(channelW, TIMER_OUTPUT_COMPARE_PWM1, whPinName);
  HT->setMode(channelW, TIMER_OUTPUT_COMPARE_PWM1, wlPinName);
  
  LL_TIM_OC_SetDeadTime(HT->getHandle()->Instance, 100); // deadtime is non linear!
  LL_TIM_CC_EnableChannel(HT->getHandle()->Instance, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
  HT->pause();
  HT->refresh();
  return HT;
}

void G431ESCDriver6PWM::pwm_set(int ulPin, uint32_t value, int resolution)
{
  PinName pin = digitalPinToPinName(ulPin);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  uint32_t index = get_timer_index(Instance);
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);

  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
  HT->setCaptureCompare(channel, value, RESOLUTION_13B_COMPARE_FORMAT);
}

void G431ESCDriver6PWM::setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) {
  //phase_state[0] = sa;
  //phase_state[1] = sb;
  //phase_state[2] = sc;
}