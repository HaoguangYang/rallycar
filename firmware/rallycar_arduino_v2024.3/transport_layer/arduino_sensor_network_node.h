#ifndef _ARDUINO_SENSOR_NETWORK_NODE_H_
#define _ARDUINO_SENSOR_NETWORK_NODE_H_

#include "node.h"

#if defined(ESP8266) or defined(ESP32) or defined(ROSSERIAL_ARDUINO_TCP)
  #include "ArduinoTcpHardware.h"
#else
  #include "ArduinoHardware.h"
#endif

namespace sensor_network {
#if defined(__AVR_ATmega8__) or defined(__AVR_ATmega168__)
/* downsize our buffers */
    #define NODE_BASE Node_<ArduinoHardware, FramerT, 8, 8, 150>
#elif defined(__AVR_ATmega328P__)
    #define NODE_BASE Node_<ArduinoHardware, FramerT, 16, 16, 256>
#elif defined(SPARK)
    #define NODE_BASE Node_<ArduinoHardware, FramerT, 20, 20, 512>
#else
    #define NODE_BASE Node_<ArduinoHardware, FramerT>
#endif
    template <typename FramerT>
    class Node : public NODE_BASE
    {
    public:
        Node(ArduinoHardware* hw, FramerT* fr, uint8_t session_id = 0) :
          NODE_BASE(hw, fr, session_id)
        {}
    };
}   /* namespace sensor_network */

#endif /* _ARDUINO_SENSOR_NETWORK_NODE_H_ */
