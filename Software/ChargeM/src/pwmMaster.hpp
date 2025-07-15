#include <Arduino.h>
#include <vector>
#include <cmath>

class basicAlg{
    private:
      static std::vector<double> inputs;
      static std::vector<double> outputs;
      static std::vector<double> cache;
      static uint32_t SampleRate;

      
    public:
      void SetPID(double Kp , double Ki, double Kd , uint32_t Samplerate){
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->SampleRate = Samplerate;
        for (int i = 0; i < 3; i++){
          cache.push_back(0);
          inputs.push_back(0);
          outputs.push_back(0);
        }
      }
      void SetSlidMode(){

      }
      
      basicAlg& AbsPID(double Target){
        double input = inputs.back();
        double error = Target - input;
        double P = Kp * error;
        double I = Ki * error + cache.back();
        double D = Kd * (input - inputs[inputs.size()-2]);
        cache.back() = error + cache.back();
        outputs.back() = P + I + D;
        inputs.back() = input;
        return *this;
      }

      basicAlg& IntervalPID(){
        
      }
      basicAlg& SlidMode(){
      }
  }
  
