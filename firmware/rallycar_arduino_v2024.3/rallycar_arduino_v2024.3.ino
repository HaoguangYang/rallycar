/**
 * The firmware running on the Arduino MCU that drives the PWM servo (steering),
 * ESC (motion), and reads the IMU sensor. The code is organized in a ROS2 style,
 * where a customized Node class is instantiated with timers and endpoints (
 * publisher/subscriber). The endpoints deal with messages -- data structures
 * automatically generated from ".msg" files (refer to message_generation.py).
 * The headers for these data structures contain data fields and serialization/
 * deserialization methods. The instantiated class contains a "spin" method,
 * that checks the buffers for message processing, as well as running functions
 * registered to timers.
 *
 * The endpoints send/receive messages through the transport layer, which utilizes
 * HDLC framing format. The HDLC frame internally contains integrety check as a
 * CRC-16 checksum at the end of the frame.
 *
 * The transport layer headers are reusable for server-side C++ programs, which
 * ensures consistent packet framing formats.
 *
 * The structure of this project is inspired by open-source projects:
 * rosserial_arduino_lib: https://github.com/frankjoshua/rosserial_arduino_lib
 * ros2arduino: https://github.com/ROBOTIS-GIT/ros2arduino
 * XRCE nano-client: https://github.com/rticommunity/nano-client
 * micro_ros_arduino: https://github.com/micro-ROS/micro_ros_arduino
 *
 * The reason why we did not directly adopt MicroROS is due to the SRAM limitation
 * on the Arduino ATMega328P MCU (2048 Bytes). This makes MicroROS unable to fit
 * on the MCU, and the bare nano-client utilizing 75%+ the memory. The old
 * rosserial_arduino_lib adopts a custom framing protocol that lacks integrety
 * check over the entire frame.
 */
#include "rallycar_driver_mcu_node.hpp"

#define RX_MTU 256

// Statically allocate serial receive buffer
static uint8_t ser_buf[RX_MTU];
// Statically instantiate the hardware abstraction layer
ArduinoHardware hw;
// Statically instantiate the framing protocol and wrapped API
sensor_network::CommsInterface<ArduinoHardware> comms(&hw);
Hdlc<sensor_network::CommsInterface<ArduinoHardware>> framer(
    &(comms.send_char), &(comms.dispatch_frame), RX_MTU, &(ser_buf[0]), &comms
);

// instantiate our custom driver class
RallycarDriverMCUNode node_inst(&hw, &framer);

void setup(){
    // Indicator of initialization status
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    // prepare hardware
    hw.setBaud(115200);
    hw.init();
    comms.register_node(&node_inst);
    node_inst.imu_init();
    node_inst.drive_by_wire_init();
    // initialization complete
    digitalWrite(LED_PIN, HIGH);
}

void loop(){
    // spin() checks for incoming data from the serial port and dispatch to endpoints
    node_inst.spin();
    // check IMU data buffer every cycle and send the data over serial when ready
    node_inst.publish_imu_when_ready();
}
