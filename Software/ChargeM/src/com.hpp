#include <Arduino.h>
#include <vector>
#include "freertos/semphr.h"

#define H2Motor 0x00
#define H2Battery 0x01
#define H2Recycling 0x02
#define H2Auxiliary 0x03
#define M2HMS 0x04
#define B2HMS 0x05
#define R2HMS 0x06
#define A2HMS 0x07
#define HMS2SlaveE 0x09
#define Slave2HMSE 0x0A 
#define HMS2Error 0x0B

#define HVCtrl 0x00
#define HVBack 0x01
#define HCSt 0x02
#define HCSp 0x03
#define HCBk 0x04
#define HBStus 0x05
#define HBalSt 0x06
#define HRStus 0x07
#define HRSt 0x08
#define HRSp 0x09
#define HAResp 0x0A

#define HAllRst 0x10
#define HEStop 0x11
#define HMConstSet 0x12
#define HBConstSet 0x13
#define HRConstSet 0x14
#define HAConstSet 0x15
#define HAllDevSet 0x16
#define HAllProSet 0x17

#define MStus 0x20
#define MConstBack 0x21
#define MEmerg 0x22
#define MDevBack 0x23

#define BStus 0x30
#define BConstBack 0x31
#define BEmerg 0x32
#define BDevBack 0x33

#define RStus 0x40
#define RConstBack 0x41
#define REmerg 0x42
#define RDevBack 0x43

#define AStus 0x50
#define AConstBack 0x51 
#define AEmerg 0x52
#define ADevBack 0x53




/* Protocol define */
/* Head 0xCC */
/* Message type: 0x00 -- HMS to motor control
                 0x01 -- HMS to battery manager
                 0x02 -- HMS to recycling system
                 0x03 -- HMS to auxiliary board
                 0x04 -- Mortor to HMS
                 0x05 -- Battery manager to HMS
                 0x06 -- Recycling system to HMS
                 0x07 -- Auxiliary board to HMS
                 0x09 -- Slave to HMS emergency signal
                 0x0A -- HMS to Slave emergency signal
                 0x0B -- Communication error
*/

/* Command:  0x00 -- velocity control(HMS sending) 
             0x01 -- velocity feedback(HMS receiving)
             0x02 -- Charging start(HMS sending)
             0x03 -- Charging stop(HMS sending)
             0x04 -- Charging feedback(HMS receiving)
             0x05 -- Battery status(HMS receiving)
             0x06 -- Battery balancing start(HMS sending)
             0x07 -- Recycling system status(HMS receiving)
             0x08 -- Recycling start(HMS sending)
             0x09 -- Recycling stop(HMS sending)
             0x0A -- Auxiliary response(HMS receiving)

             0x10 -- All reset(HMS sending)
             0x11 -- Emergency stop(HMS sending)
             0x12 -- Motor control parameters setting(HMS sending)
             0x13 -- Battery manager parameters setting(HMS sending)
             0x14 -- Recycling system parameters setting(HMS sending)
             0x15 -- Auxiliary board parameters setting(HMS sending)
             0x16 -- All set to development mode(HMS sending)
             0x17 -- All set to production mode(HMS sending)

             0x20 -- Motor control status feedback(Motor control sending)
             0x21 -- Motor control parameters feedback(Motor control sending)
             0x22 -- Emergency signal feedback(Motor control sending)
             0x23 -- Development mode feedback(Motor control sending)

             0x30 -- Battery manager status feedback(Battery manager sending)
             0x31 -- Battery manager parameters feedback(Battery manager sending)
             0x32 -- Battery manager emergency signal feedback(Battery manager sending)
             0x33 -- Development mode feedback(Battery manager sending)

             0x40 -- Recycling system status feedback(Recycling system sending)
             0x41 -- Recycling system parameters feedback(Recycling system sending)
             0x42 -- Recycling system emergency signal feedback(Recycling system sending)
             0x43 -- Development mode feedback(Recycling system sending)

             0x50 -- Auxilary system status feedback(Auxilary system sending)
             0x51 -- Auxilary system parameters feedback(Auxilary system sending)
             0x52 -- Auxilary system emergency signal feedback(Auxilary system sending)
             0x53 -- Development mode feedback(Auxilary system sending)
*/

/* Data lenth in bytes 0x?? */

/* Data 
Each message frame only contains one type of data. Won's from different module at a same frame.

Motor feed back format
Current velocity : 4 bytes(float)
Current power : 4 bytes(float)
Current voltage : 4 bytes(float)
Current current : 4 bytes(float)
Current temperature1 : 4 bytes(float)
Current temperature2 : 4 bytes(float)
Development mode :
Current phaseU voltage : 4 bytes(float)
Current phaseV voltage : 4 bytes(float)
Current phaseW voltage : 4 bytes(float)
Current phaseU current : 4 bytes(float)
Current phaseV current : 4 bytes(float)
Current phaseW current : 4 bytes(float)
Current Speed error : 4 bytes(float)
Current Position : 4 bytes(float)

Battery manager feed back format
Current status : 1 byte: 0x00 -- Normal
                        0x01 -- Charging
                        0x02 -- Recycling
                        0x03 -- Fault
Current voltage : 4 bytes(float)
Current current : 4 bytes(float)
Current power amount : 4 bytes(float)
Current temperature1 : 4 bytes(float)
Current temperature2 : 4 bytes(float)
Current charge cycles : 4 bytes(uint32_t)
Current charge current : 4 bytes(float)
Battery capacity : 4 bytes(float)
Battery balancing status : 1 byte:  0x00 -- Need balancing
                                    0x01 -- Balancing
                                    0x02 -- No need balancing
Development mode :
Current cell voltage : 4 bytes(float)
Current parameters : n*4 bytes(float)

Recycling system feed back format
Current status : 1 byte: 0x00 -- No recycling
                        0x01 -- Recycling
                        0x02 -- Fault
Current voltage : 4 bytes(float)
Current current : 4 bytes(float)
Total generated power : 4 bytes(float)
Current PWM percentage : 4 bytes(float)
Current mosfet temperature : 4 bytes(float)
Current resistance temperature : 4 bytes(float)
Current RPM : 4 bytes(float)
Development mode :
Current parameters : n*4 bytes(float)

Auxiliary board feed back format
Current status : 1 byte: 0x00 -- Normal
                        0x01 -- Fault
Current direction : 1 byte: 0x00 -- To motor
                        0x01 -- To battery
                        0x02 -- No direction
Current bus voltage : 4 bytes(float)
Current bus current : 4 bytes(float)
Current bus power : 4 bytes(float)
Sensor 1: name : 1 byte
            value : 4 bytes(float)
Sensor 2: name : 1 byte
            value : 4 bytes(float)
Sensor 3: name : 1 byte
            value : 4 bytes(float)
.....

*/

/* END 
End by : 0xFF 0xFF -- End of message
*/

/* Verification 
Odd verify : 1 byte
*/

// Total message structure
// 1 byte // Head
// 1 byte // Message type
// 1 byte // Command
// 1 byte // Data length
// n bytes // Data
// 2 bytes // End
// 1 byte // Verification
// Maximum length is 256 bytes


class com{
    private:
        static SemaphoreHandle_t mutex;
        static uint8_t Channel;
    protected:
        static std::vector<byte> buffer;
        static std::vector<byte> Command;
        static uint8_t RxPin;
        static uint8_t TxPin;

        /**
         * @brief Initialize. Channel default is 0
         * 
         * @param rxPin 
         * @param txPin 
         * @param SerialChannel 
         */
        void init(uint8_t rxPin, uint8_t txPin , uint8_t SerialChannel = 0) {
            Serial.begin(115200, SERIAL_8N1, rxPin, txPin);
            Channel = 0;
            if(SerialChannel == 0) {
                Serial1.begin(115200, SERIAL_8N1, rxPin, txPin);
                Channel = 1;
            }
            mutex = xSemaphoreCreateMutex();
            RxPin = rxPin;
            TxPin = txPin;
        }
        /**
         * @brief Very simple verification. Simply calculate how many odd numbers in the command vector and return the sum.
         * 
         * @return byte 
         */
        byte OddVerify(std::vector<byte> &Cmd , uint16_t till = 0){
            uint sum = 0;
            for(uint8_t i = 0; i < Cmd.size() - till ; i++){
                uint8_t odd = Cmd[i] / 2;
                sum += odd;
            }
            return sum;
        }



        void SendCommand(){
            if(xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE){
                Serial.println("com: Failed to take mutex");
                return;
            }
            switch(Channel) {
                case 0:
                    Serial.write(Command.data(), Command.size());
                    break;
                case 1:
                    Serial1.write(Command.data(), Command.size());
                    break;
                default:
                    Serial.println("com: Invalid channel");
                    break;
            }
            xSemaphoreGive(mutex);
        }

        /**
         * @brief ReceiveCommand will read data from the serial port and verify the data frame. Save the data to the buffer.
         * 
         * @return true 
         * @return false 
         */
        bool ReceiveCommand(){
            bool ifDataFrame;
            if(xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE){
                Serial.println("com: Failed to take mutex");
                return;
            }
            switch (Channel) {
                case 0:
                    while(Serial.available() > 0) {
                        if(Serial.peek() != 0xCC && ifDataFrame == false) {
                            Serial.read(); // Skip invalid data
                            continue;
                        }else if(Serial.peek() == 0xCC) {
                            ifDataFrame = true; // Start of a new data frame
                        }
                        buffer.push_back(Serial.read());
                    }
                    ifDataFrame = false; // Reset for next frame
                    break;
                case 1:
                    while(Serial1.available() > 0) {
                        if(Serial1.peek() != 0xCC && ifDataFrame == false) {
                            Serial1.read(); // Skip invalid data
                            continue;
                        }else if(Serial1.peek() == 0xCC) {
                            ifDataFrame = true; // Start of a new data frame
                        }
                        buffer.push_back(Serial1.read());
                    }
                    ifDataFrame = false; // Reset for next frame
                    break;
                default:
                    Serial.println("com: Invalid channel");
                    break;
            }

            if(buffer.size() < 5) { // Minimum size is 5 bytes (0xCC, type, command, length, 0xFF)
                xSemaphoreGive(mutex);
                return false; // Not enough data
            }
            uint8_t oddSum = buffer.back();
            uint8_t verify = OddVerify(buffer, 1); // Exclude the last byte (verification byte)
            if(oddSum != verify) {
                Serial.println("com: Verification failed");
                buffer.clear();
                xSemaphoreGive(mutex);
                return false; // Verification failed
            }

            return true; // Data received and verified successfully
            xSemaphoreGive(mutex);
        }
        public:
        /**
         * @brief Turn any type of data to byte array.
         * 
         * @tparam T 
         * @param Value 
         * @return uint8_t* 
         */
        template <typename T>
            uint8_t* ToByte(T Value){
                uint8_t* byteArray = new uint8_t[sizeof(T)];
                memcpy(byteArray, &Value, sizeof(T));
                return byteArray;
            }
    };

    class AppllyOnProtocol: public com {
        private:
            static SemaphoreHandle_t Amutex;
        
        public:
            std::vector<float> Recieve(uint8_t Info[3]){
                if(!ReceiveCommand()) {
                    Serial.println("com: Failed to receive command");
                    return {};
                }
                std::vector<float> data;
                if(xSemaphoreTake(Amutex, portMAX_DELAY) != pdTRUE){
                    Serial.println("com: Failed to take mutex");
                    return {};
                }
                if(buffer.size() < 5) {
                    Serial.println("com: Buffer size is less than 5");
                    xSemaphoreGive(Amutex);
                    return {};
                }
                *Info = buffer[1]; // Message type
                *(Info + 1) = buffer[2]; // Command
                *(Info + 2) = buffer[3]; // Data length

                for(uint8_t i = 4; i < buffer.size() - 2; i += 4) {
                    float value;
                    memcpy(&value, &buffer[i], sizeof(float));
                    data.push_back(value);
                }
                buffer.clear(); // Clear the buffer after reading
                xSemaphoreGive(Amutex);
                return data; // Return the data read from the buffer
            }

            /**
             * @brief Send data to slave or master. Universal function
             * 
             * @param type Refer to the macro definitions for message type
             * @param Commands Refer to the macro definitions for command
             * @param data Float data to send.
             * @return true 
             * @return false 
             */
            bool Send(uint8_t type , uint8_t Commands , float *data){
                if(xSemaphoreTake(Amutex, portMAX_DELAY) != pdTRUE){
                    Serial.println("com: Failed to take mutex");
                    return false;
                }
                Command.clear();
                Command.push_back(0xCC); // Head
                Command.push_back(type); // Message type
                Command.push_back(Commands); // Command
                uint8_t dataLength = sizeof(data);
                Command.push_back(dataLength); // Data length

                uint8_t* ByteArray = new uint8_t[dataLength * sizeof(float)];
                for(int i = 0; i < dataLength / 4; i++) {
                    uint8_t* temp = new uint8_t[4];
                    temp = ToByte(data[i]);
                    memcpy(ByteArray + i * sizeof(float), temp, sizeof(float));
                    delete[] temp; // Free the temporary byte array
                }

                for(uint8_t i = 0; i < dataLength; i++) {
                    Command.push_back(ByteArray[i]);
                }
                delete[] ByteArray; // Free the byte array after use
                uint8_t verify = OddVerify(Command);
                Command.push_back(0xFF); // Add verification byte
                Command.push_back(verify); // Add verification byte
                SendCommand(); // Send the command
                Command.clear(); // Clear the command after sending
                xSemaphoreGive(Amutex);
                return true; // Return true if command sent successfully
            }



    };