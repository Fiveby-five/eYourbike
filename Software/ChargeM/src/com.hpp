#include <Arduino.h>
#include <vector>
#include "freertos/semphr.h"

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
        static std::vector<byte> buffer;
        static uint8_t Channel;
    public:
        static std::vector<byte> Command;

        void init(uint8_t rxPin, uint8_t txPin , uint8_t SerialChannel = 0) {
            Serial.begin(115200, SERIAL_8N1, rxPin, txPin);
            Channel = 0;
            if(SerialChannel == 0) {
                Serial1.begin(115200, SERIAL_8N1, rxPin, txPin);
                Channel = 1;
            }
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

        void LoadCommand(std::vector<byte> &cmd){
            if(xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE){
                Serial.println("com: Failed to take mutex");
                return;
            }
            Command.push_back(0xCC); // Head
            for(uint8_t i = 0; i < cmd.size(); i++){
                Command.push_back(cmd[i]);
            }
            Command.push_back(0xFF); // End
            byte verify = OddVerify(Command);
            Command.push_back(verify); // Verification

            xSemaphoreGive(mutex);
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
            Command.clear();
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
    };

    class AppllyOnProtocol: public com {
        private:
        public:
    };