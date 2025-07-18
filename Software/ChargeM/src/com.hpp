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

*/


class com{
    private:
        static SemaphoreHandle_t mutex;
        static std::vector<char> buffer;
    public:
        static std::vector<char> Command;

        static void init(uint8_t rxPin, uint8_t txPin , uint8_t SerialChannel = 0) {
            Serial.begin(115200, SERIAL_8N1, rxPin, txPin);
            if(SerialChannel == 0) {
                Serial1.begin(115200, SERIAL_8N1, rxPin, txPin);
            }

        }
};