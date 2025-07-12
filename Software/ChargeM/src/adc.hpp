#include <Arduino.h>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define Pi 3.141592f


class ADC : public ADCFilter{
    private:
        static uint64_t pTmsV;
        static uint64_t pTmsI;
        static uint8_t BufferIndex;
        ADCFilter& filter;
        static bool isVStable;
        static bool isIStable;
        SemaphoreHandle_t mutex;
       
    public:
        ADC(ADCFilter& filter) : filter(filter) {
            mutex = xSemaphoreCreateMutex();
        }

        ~ADC() {
            vSemaphoreDelete(mutex);
        }



    /** 
    * @brief Initialize the ADC
    * @param Vpin The pin to read the voltage
    * @param Ipin The pin to read the current
    */
        ADC& init(uint8_t vPin, uint8_t iPin = 255){
            pinMode(vPin, INPUT);
            if(iPin != 255) {
                pinMode(iPin, INPUT);
            }
        }

        /**
         * @brief Get the Vbus object. This function reads the voltage from the ADC and calculates the sampling rate automatically.
         * 
         * @param dt Delay in microseconds between readings. Default is 0.
         * @return ADC&  Reference to the ADC object for chaining
         */
        ADC& getVbus(uint32_t dt = 0){
            if(xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE){
                Serial.println("ADC: Failed to take mutex");
                return *this;
            }
            uint32_t Sdt;
            filter.VoltOrCurrent = 0;
            if(isVStable != true){
                uint32_t* TestBuffer = new uint32_t[64];
                pTmsV = micros();
                for (int i = 0 ; i < 64; i++) {
                    TestBuffer[i] = analogReadMilliVolts(Vpin);
                    if(filter.delayTus > 0) {
                        delayMicroseconds(dt);
                    }
                    Sdt += micros() - pTmsV;
                    pTmsV = micros();
                }
                sampleRate = 1000000 / Sdt / bufferSize;
                delete[] TestBuffer;
                isVStable = true;
            }
            else{
                filter.bufferV.push_back(analogReadMilliVolts(filter.Vpin));
                filter.bufferV.erase(filter.bufferV.begin());
                if(filter.delayTus > 0){
                    delayMicroseconds(dt);
                }
            }
            xSemaphoreGive(mutex);
            return *this;
        }


        /** Get Ibus
         * @brief Get the current from the ADC. The sampling rate will be calculated automatically.
         * @param dt Delay in microseconds between readings. Default is 0.
         * @return ADC& Reference to the ADC object for chaining
         */
        ADC& getIbus(uint32_t dt = 0){
            if(xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE){
                Serial.println("ADC: Failed to take mutex");
                return *this;
            }
            uint32_t Sdt;
            filter.VoltOrCurrent = 1;
            if(isIStable != true){
                uint32_t* TestBuffer = new uint32_t[64];
                pTmsI = micros();
                for (int i = 0 ; i < 64; i++) {
                    TestBuffer[i] = analogReadMilliVolts(Ipin);
                    if(filter.delayTus > 0) {
                        delayMicroseconds(dt);
                    }
                    Sdt += micros() - pTmsI;
                    pTmsI = micros();
                }
                sampleRate = 1000000 / Sdt / bufferSize;
                delete[] TestBuffer;
                isIStable = true;
            }
            else{
                filter.bufferI.push_back(analogReadMilliVolts(filter.Ipin));
                filter.bufferI.erase(filter.bufferI.begin());
                if(filter.delayTus > 0){
                    delayMicroseconds(dt);
                }
            }
            xSemaphoreGive(mutex);
            return *this;
        }
        
};

class ADCFilter {
    protected:
        static uint32_t sampleRate;
        static std::vector<uint32_t> bufferV;
        static std::vector<uint32_t> bufferI;
        static uint32_t bufferSize;
        static uint32_t delayTus;
        static uint8_t Vpin;
        static uint8_t Ipin;
        static bool VoltOrCurrent;

        typedef struct {
            static float b0, b1 ,b2;
            static float a1, a2;
            static float x1, x2;
            static float y1, y2;
            static uint32_t CenterFreq;
            static uint32_t Bandwidth;
            static bool isInit;
        } BandPassParameters;

        typedef struct {
            static float alpha;
            static float tau;
            static uint32_t CutOffFreq;
            static float dt;
            static uint32_t PrevOut;
            static bool isInit;
        } LowPassParameters;

        BandPassParameters BPPara;
        LowPassParameters LPPara;

    public:

        /**
         * @brief Set the filter parameters for Low-pass and Band-pass filters. If you want to use low-pass filter, set CutOffFreq , set buffer size more than 2. 
         * If you want to use band-pass filter, set CenterFreq and Bandwidth , set buffer size more than 3.
         * If you only need to read data without filtering, set all parameters to 0 and set buffer size to 1.
         * 
         * 
         * @param CutOffFreq Cut-off frequency for Low-pass filter
         * @param CenterFreq Center frequency for Band-pass filter
         * @param Bandwidth Bandwidth for Band-pass filter
         * @param bufferSize Size of the buffer to hold filtered data
         */
        void SetFilterParams(uint32_t CutOffFreq = 0, uint32_t CenterFreq = 0, uint32_t Bandwidth = 0, uint8_t bufferSize) {
            if(CutOffFreq > 0) {
                LPPara.CutOffFreq = CutOffFreq;
                LPPara.tau = 1.0f / (2.0f * Pi * LPPara.CutOffFreq);
            }
            if(CenterFreq > 0) {
                BPPara.CenterFreq = CenterFreq;
                BPPara.Bandwidth = Bandwidth;
                BPPara.b1 = 0.0f;
            }
            for (int i = 0; i < bufferSize; i++) {
                bufferV.push_back(0);
                bufferI.push_back(0);
            }
        }

        /**
         * @brief Low-pass filter implementation. Use this after setting voltage or current data. Use as chainable function.
         * 
         * @return uint32_t 
         */
        uint32_t LowPassFilter(){
            uint32_t input;
            if(!LPPara.isInit) {
                LPPara.isInit = true;
                LPPara.dt = 1.0f / sampleRate;
                LPPara.alpha = LPPara.dt / (LPPara.tau + LPPara.dt);
                return 0;
            }

            if(VoltOrCurrent == 0) {
                input = bufferV.back();
            } else {
                input = bufferI.back();
            }
            uint32_t output = LPPara.PrevOut + LPPara.alpha * (input - LPPara.PrevOut);
            LPPara.PrevOut = output;
            return output;
        }


        /**
         * @brief Band-pass filter implementation. Use this after setting voltage or current data. Use as chainable function.
         * 
         * @return uint32_t 
         */
        uint32_t BandPassFilter(){
            uint32_t input;
            if(VoltOrCurrent == 0) {
                input = bufferV.back();
            } else {
                input = bufferI.back();
            }
            if(!BPPara.isInit) {
                BPPara.isInit = true;
                float omega = 2.0f * Pi * BPPara.CenterFreq;
                float Q = BPPara.CenterFreq / BPPara.Bandwidth;

                float T = 1.0f / sampleRate;
                float K = omega / Q;
                float C = pow(omega / Q * T / 2.0f , 2);
                float D = omega /Q * T / 2.0f;
                float denominator = 1.0f + 2.0f * D + C + pow(D, 2);

                BPPara.b0 = K * 4.0f * D / denominator;
                BPPara.b1 = 0.0f;
                BPPara.b2 = -BPPara.b0;
                BPPara.a1 = 2.0f * (C - 1.0f - pow(D, 2)) / denominator;
                BPPara.a2 = (1.0f - 2.0f * D + C - pow(D, 2)) / denominator;

                if(VoltOrCurrent == 0) {
                    BPPara.x1 = bufferV[bufferSize - 2];
                    BPPara.x2 = bufferV[bufferSize - 3];
                    BPPara.y1 = 0;
                    BPPara.y2 = 0;
                } else {
                    BPPara.x1 = bufferI[bufferSize - 2];
                    BPPara.x2 = bufferI[bufferSize - 3];
                    BPPara.y1 = 0;
                    BPPara.y2 = 0;
                }
                return 0;
            }

            float output = BPPara.b0 * input + BPPara.b1 * BPPara.x1 + BPPara.b2 * BPPara.x2 - BPPara.a1 * BPPara.y1 - BPPara.a2 * BPPara.y2;

            BPPara.x2 = BPPara.x1;
            BPPara.x1 = input;
            BPPara.y2 = BPPara.y1;
            BPPara.y1 = output;

            return static_cast<uint32_t>(output);

        }
};