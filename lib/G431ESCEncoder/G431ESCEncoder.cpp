#include "G431ESCEncoder.h"

/*
  HardwareEncoder(int cpr)
*/



G431ESCEncoder::G431ESCEncoder(uint32_t _ppr, bool useIndex) {
    rotations_per_overflow = 10;
   

    overflow_count = 0;
    count = 0;
    prev_count = 0;
    prev_overflow_count = 0;
    pulse_timestamp = 0;

    cpr = _ppr * 4; // 4x for quadrature
    ppr = _ppr;
    ticks_per_overflow = cpr * 10;

    // velocity calculation variables
    prev_timestamp = getCurrentMicros();
    pulse_timestamp = getCurrentMicros();  
}



/*
  Shaft angle calculation
*/
float G431ESCEncoder::getSensorAngle() { return getAngle(); }

float G431ESCEncoder::getMechanicalAngle() {
    return _2PI * (count % static_cast<int>(cpr)) / static_cast<float>(cpr);
}

float G431ESCEncoder::getAngle() {
    update();
    return _2PI * (count / static_cast<float>(cpr) +
                   overflow_count * rotations_per_overflow);
}

double G431ESCEncoder::getPreciseAngle() {
    return _2PI * (count / static_cast<double>(cpr) +
                   overflow_count * rotations_per_overflow);
}


int32_t G431ESCEncoder::getFullRotations() {
    return count / static_cast<int>(cpr) +
           overflow_count * rotations_per_overflow;
}


uint16_t G431ESCEncoder::getEncoderCount(){
    update();
    return count % cpr;
}

double G431ESCEncoder::getEncoderAngle() {
    return 360.0 * (cpr / static_cast<double>(getEncoderCount())); 
}


void G431ESCEncoder::resetCounts() {
    (TIM4->CNT) = 0;
    overflow_count = 0;
    prev_overflow_count = 0;
    prev_timestamp = getCurrentMicros();
    pulse_timestamp = prev_timestamp;  
    update();
}


/*
  Shaft velocity calculation
*/
float G431ESCEncoder::getVelocity() {
    // sampling time calculation
    float dt = (pulse_timestamp - prev_timestamp) * 1e-6f;
    // quick fix for strange cases (micros overflow)
    if (dt <= 0 || dt > 0.5f)
        dt = 1e-3f;

    // time from last impulse
    int32_t overflow_diff = overflow_count - prev_overflow_count;
    int32_t dN = (count - prev_count) + (ticks_per_overflow * overflow_diff);

    float pulse_per_second = dN / dt;

    // velocity calculation
    return pulse_per_second / (static_cast<float>(cpr)) * _2PI;
}

// getter for index pin
int G431ESCEncoder::needsSearch() { return false; }

// private function used to determine if encoder has index
int G431ESCEncoder::hasIndex() { return 0; }

// encoder initialisation of the hardware pins
// and calculation variables
void G431ESCEncoder::init() {
    // overflow handling
    rotations_per_overflow = 10;
    ticks_per_overflow = cpr * rotations_per_overflow;

    
    // set up GPIO
    EncoderInit();
    //TIM4->CNT = 32768;
    // counter setup
    overflow_count = 0;
    count = 0;
    prev_count = 0;
    prev_overflow_count = 0;

    prev_timestamp = getCurrentMicros();
    pulse_timestamp = getCurrentMicros();

   
    

}

void G431ESCEncoder::update() {
    // handle overflow of the 16-bit counter here
    // must be called at least twice per traversal of overflow range
    // TODO(conroy-cheers): investigate performance impact
    prev_count = count;
    count = (TIM4->CNT);

    prev_timestamp = pulse_timestamp;
    pulse_timestamp = getCurrentMicros();

    prev_overflow_count = overflow_count;
}




void G431ESCEncoder::handleOverflow(void){
    if((GETBIT(TIM4->CR1,4))){
        overflow_count++;
    }else{
        overflow_count--;
    }
}

