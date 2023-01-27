#ifndef __G431ESCENC_H__
#define __G431ESCENC_H__

#include "common/base_classes/Sensor.h"
#include "g431timer.h"
#include <HardwareTimer.h>

#define GETBIT(var, bit)	(((var) >> (bit)) & 1)



class G431ESCEncoder : public Sensor {
public:
	explicit G431ESCEncoder(uint32_t _ppr = 600, bool useIndex = false);
    /** encoder initialise pins */
    void init() override;
    // Encoder configuration
    uint32_t cpr;  //!< encoder cpr number
    uint32_t ppr;  //!< encoder cpr number
    // Abstract functions of the Sensor class implementation
    /** get current angle (rad) */
    float getSensorAngle() override;
    float getMechanicalAngle() override;
    /**  get current angular velocity (rad/s) */
    float getVelocity() override;
    float getAngle() override;
    double getPreciseAngle() override;
    int32_t getFullRotations() override;
    
	
    /**
     * returns 0 if it does need search for absolute zero
     * 0 - encoder without index
     * 1 - ecoder with index
     */
    int needsSearch() override;
	void handleOverflow();
	void update() override;


    //added personal methods
    uint16_t getEncoderCount();
    double getEncoderAngle();
    void resetCounts();

  private:
    int hasIndex();  // !< function returning 1 if encoder has index pin and 0 if not.

    

    TIM_HandleTypeDef encoder_handle;

    u_int16_t rotations_per_overflow;
    u_int16_t ticks_per_overflow;

    volatile int32_t overflow_count;
    volatile u_int16_t count;  //!< current pulse counter
    volatile u_int16_t prev_count;
    volatile int32_t prev_overflow_count;

    // velocity calculation variables
    volatile int32_t pulse_timestamp, prev_timestamp;
};
#endif