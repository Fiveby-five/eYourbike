#include "adc.hpp"

void ADC::init(uint8_t vPin, uint8_t iPin , uint8_t BufferSize){
    Vpin = vPin;
    Ipin = iPin;
    bufferSize = BufferSize;

}