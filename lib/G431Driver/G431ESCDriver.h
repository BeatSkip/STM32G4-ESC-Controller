#ifndef G431Driver_h
#define G431Driver_h

#include <SimpleFOC.h>
#include <HardwareTimer.h>

/**
 6 pwm bldc driver class
*/
class G431ESCDriver6PWM: public BLDCDriver
{
  public:
    /**
      BLDCDriver class constructor
      @param phA_h A phase pwm pin
      @param phA_l A phase pwm pin
      @param phB_h B phase pwm pin
      @param phB_l A phase pwm pin
      @param phC_h C phase pwm pin
      @param phC_l A phase pwm pin
      @param en enable pin (optional input)
    */
    G431ESCDriver6PWM(uint32_t pwmfreq, uint32_t dead_time, float psu, float limit);
    
    /**  Motor hardware init function */
  	int init() override;
    /** Motor disable function */
  	void disable() override;
    /** Motor enable function */
    void enable() override;

    // hardware variables
  	int pwmA_h,pwmA_l; //!< phase A pwm pin number
  	int pwmB_h,pwmB_l; //!< phase B pwm pin number
  	int pwmC_h,pwmC_l; //!< phase C pwm pin number

    const int IN1_H = A_PHASE_UH;
    const int IN1_L = A_PHASE_UL;
    const int IN2_H = A_PHASE_VH;
    const int IN2_L = A_PHASE_VL;
    const int IN3_H = A_PHASE_WH;
    const int IN3_L = A_PHASE_WL;

    int enable_pin; //!< enable pin number
    bool enable_active_high = true;

    uint32_t dead_zone; //!< a percentage of dead-time(zone) (both high and low side in low) for each pwm cycle [0,1]

    /** 
     * Set phase voltages to the harware 
     * 
     * @param Ua - phase A voltage
     * @param Ub - phase B voltage
     * @param Uc - phase C voltage
    */
    void setPwm(float Ua, float Ub, float Uc) override;

    /** 
     * Set phase voltages to the harware 
     * 
     * @param sc - phase A state : active / disabled ( high impedance )
     * @param sb - phase B state : active / disabled ( high impedance )
     * @param sa - phase C state : active / disabled ( high impedance )
    */
    virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) override;
    void pwm_manual(uint16_t Ua, uint16_t Ub, uint16_t Uc);
  private:

    void pwm_set(int ulPin, uint32_t value, int resolution);
    HardwareTimer* pwm_init_high_and_low(int uhPin, int ulPin, int vhPin, int vlPin, int whPin, int wlPin,uint32_t PWM_freq);

    HardwareTimer *MyTim1;
    uint32_t channel1,channel2,channel3; 
        
};


#endif