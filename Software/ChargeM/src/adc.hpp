#include <Arduino.h>
#include <vector>
#include "Bus.hpp"

#define Pi 3.141592f


class ADC {
    private:
        static std::vector<uint32_t> Buffer;
        static uint8_t Vpin;
        static uint8_t Ipin;
        static uint8_t bufferSize;
    public:
    /** 
    * @brief Initialize the ADC
    * @param Vpin The pin to read the voltage
    * @param Ipin The pin to read the current
    * @param bufferSize The size of the buffer
    */
        void init(uint8_t vPin, uint8_t iPin = 255, uint8_t BufferSize = 10);
        /**
         * @brief Get the Vbus object
         * 
         * @param FilterType 0: No filter, 1: Average, 2: LowPass, 3: BandPass
         * @return uint32_t 
         */
        uint32_t getVbus(uint8_t FilterType = 0);
        /**
         * @brief Get the Ibus object
         * 
         * @param FilterType 0: No filter, 1: Average, 2: LowPass, 3: BandPass
         * @return uint32_t 
         */
        uint32_t getIbus(uint8_t FilterType = 0);
        
        /**
         * @brief  Just average
         */
        uint32_t Average();

        /**
         * @brief 1st order low pass filter
         * @param cutOffFreq The cutoff frequency
         * @param Gain The gain constant, default is 1
         * @return Result data
         */
        uint32_t LowPass(uint32_t cutOffFreq , float Gain = 1.0f);

        /**
         * @brief 

         * @param CenterFreq The center frequency
         * @param Bandwidth The bandwidth
         * @param SampleRate The sample rate, set to 0 for auto calculate
         * @return Result data
         */
        uint32_t BandPass(uint32_t CenterFreq, uint32_t Bandwidth, uint32_t SampleRate);
};