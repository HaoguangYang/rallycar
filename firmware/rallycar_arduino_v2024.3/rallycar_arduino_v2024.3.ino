#include "rallycar_driver_mcu_node.hpp"

#define RX_MTU 256

static uint8_t ser_buf[RX_MTU];
ArduinoHardware hw;
sensor_network::CommsInterface<ArduinoHardware> comms(&hw);
Hdlc<sensor_network::CommsInterface<ArduinoHardware>> framer(
    &(comms.send_char), &(comms.dispatch_frame), RX_MTU, &(ser_buf[0]), &comms
);

RallycarDriverMCUNode node_inst(&hw, &framer);

void setup(){
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    hw.setBaud(115200);
    hw.init();
    comms.register_node(&node_inst);
    node_inst.imu_init();
    node_inst.drive_by_wire_init();
    digitalWrite(LED_PIN, HIGH);
}

void loop(){
    node_inst.spin();
    node_inst.publish_imu_when_ready();
}
