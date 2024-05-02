#ifndef _SERIAL_COMMS_INTERFACE_HPP_
#define _SERIAL_COMMS_INTERFACE_HPP_

// reused data structures from bare-metal application
#include "../../firmware/rallycar_arduino_v2024.3/builtin_interfaces/msg/time.h"
#include "../../firmware/rallycar_arduino_v2024.3/transport_layer/endpoint.h"
#include "../../firmware/rallycar_arduino_v2024.3/transport_layer/hdlc.h"
// for serial communication in Linux
#include <chrono>
#include <cstdlib>
#include <future>
#include <thread>

#include "serial.h"
#include "serial_impl.h"

/**
 * @brief This template class provides a server-side "shim" that talks to the client-side
 * (defined in firmware/rallycar_arduino_v2024.3/transport_layer/endpoint.h), over a serial
 * port. After setting port and baudrate, the user may call the "transferInit" method of
 * this template class, to initialize the handshake with the client and start talking.
 *
 * Data structures are packaged with its own headers, internally managed as "endpoints",
 * which follows a publish/subscribe model. User need to register endpoints with a specific
 * data type, integer ID, and (for "subscriber" endpoint) callback function upon receving.
 * User-space functions can get the write buffer associated with a "publisher" endpoint,
 * write and then return it, for the data to be sent on the serial port. The template class
 * initializes a listener for each "subscriber" endpoint, deserializes and dispatches the
 * incoming information to the corresponding endpoint.
 *
 * Send and receive buffers and endpoint list are allocated statically.
 *
 * @tparam RX_MTU Receive buffer length
 * @tparam TX_MTU Send buffer length
 * @tparam MAX_ENDPOINTS Max number of registered endpoints
 */
template <uint16_t RX_MTU = 256, uint16_t TX_MTU = 256, uint8_t MAX_ENDPOINTS = 64>
class SerialCommsInterface {
  using SerialEndpointBase = sensor_network::EndpointBase;

 private:
  serial::Serial ser;
  Hdlc<SerialCommsInterface> framer;

  uint8_t rxBuffer[RX_MTU];
  uint8_t txBuffer[TX_MTU];
  // because we are running two threads here, we need to account for race conditions.
  // allocate a small private buffer specialized for time sync and reset
  uint8_t privTxBuffer[24];
  bool txBufferLocked = false;
  SerialEndpointBase* endpoints[MAX_ENDPOINTS] = {nullptr};
  uint8_t max_stream_record = 0;
  std::promise<void> exit_signal_;
  std::shared_future<void> future_;
  std::thread ser_read_thread_;

  NodeState driverState = NodeState::OUT_OF_SYNC;

  void reset() {
    // this is not thread-safe
    // uint8_t* buf = getEndpointBuffer((uint8_t)NodeServiceID::RESET_ID);
    // *buf = 0;
    // returnEndpointBuffer(1);

    // thread-safe version
    privTxBuffer[0] = session_id;
    privTxBuffer[1] = (uint8_t)NodeServiceID::RESET_ID;
    privTxBuffer[2] = 0;
    framer.sendFrame((const char*)(&privTxBuffer[0]), 3);

    driverState = NodeState::OUT_OF_SYNC;
  }

  bool clock_sync_reply(const uint8_t* req, const uint16_t& length) {
    static SerialTime t;
    // if (length != SerialTime::deserialize(t, req))... check frame length for data integrity
    if (length != 8) return false;

    // construct reply message
    // this is not thread-safe
    // uint8_t* txBuff = getEndpointBuffer((uint8_t)NodeServiceID::TIME_SYNC_ID);
    // if (txBuff == nullptr) return false;

    // thread-safe version
    privTxBuffer[0] = session_id;
    privTxBuffer[1] = (uint8_t)NodeServiceID::TIME_SYNC_ID;
    uint8_t* txBuff = &privTxBuffer[2];

    t = time_now();
    uint8_t* ptr = txBuff;
    for (uint16_t n = 0; n < length; n++) {
      // part 1: repeat input timestamp
      *ptr = req[n];
      ptr++;
    }
    ptr += SerialTime::serialize(t, ptr);

    // returnEndpointBuffer(ptr - txBuff);
    framer.sendFrame((const char*)(&privTxBuffer[0]), ptr - txBuff + 2);

    return true;
  }

  void spin(const std::shared_future<void>& local_future) {
    std::future_status status;
    static uint8_t data;
    do {
      if (ser.read(&data, 1) == 1) framer.receiveChar(data);
      status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
  }

  void send_char(const uint8_t& data) { ser.write(&data, 1); }

  void onIncomingData(const uint8_t* payload, const uint16_t& length) {
    if (payload[0] != session_id || length <= 2) return;
    switch (payload[1]) {
      case (uint8_t)NodeServiceID::TIME_SYNC_ID:
        if (clock_sync_reply(&payload[2], length - 2)) driverState = NodeState::RUNNING;
        return;
      default:
        if (driverState != NodeState::RUNNING || payload[1] >= max_stream_record) return;
        break;
    }
    endpoints[payload[1]]->spin(&payload[2], length - 2);
  }

 protected:
  using SerialTime = builtin_interfaces__msg::Time;
  virtual SerialTime time_now() = 0;

 public:
  const uint8_t session_id;

  SerialCommsInterface(const uint8_t& sessionId = 0, const std::string& port = "",
                       const uint32_t& baud = 115200)
      : ser{port, baud, serial::Timeout::simpleTimeout(1000)},
        framer{&SerialCommsInterface::send_char, &SerialCommsInterface::onIncomingData, RX_MTU,
               &(rxBuffer[0]), this},
        future_{exit_signal_.get_future()},
        session_id(sessionId) {}

  ~SerialCommsInterface() {
    if (ser.isOpen()) {
      reset();
      exit_signal_.set_value();
      ser_read_thread_.join();
      ser.close();
    }
  }

  void setPort(const std::string& port) { ser.setPort(port); }
  void setBaudrate(const uint32_t& baud) { ser.setBaudrate(baud); }

  bool transferInit() {
    if (ser.isOpen()) ser.close();
    try {
      ser.open();
      ser_read_thread_ =
          std::thread(&SerialCommsInterface<RX_MTU, TX_MTU, MAX_ENDPOINTS>::spin, this, future_);
      return true;
    } catch (const std::exception& e) {
      printf("ERROR occured when trying to open serial port: %s\n", ser.getPort().c_str());
      printf("    Backtrace: %s\n", e.what());
      return false;
    }
  }

  bool registerEndpoint(SerialEndpointBase* endpoint, const uint8_t& stream_id) {
    if (stream_id >= MAX_ENDPOINTS) return false;
    if (endpoints[stream_id] != nullptr) return false;
    endpoints[stream_id] = endpoint;
    endpoints[stream_id]->stream_id = stream_id;
    if (stream_id >= max_stream_record) max_stream_record = stream_id + 1;
    return true;
  }

  uint8_t* getEndpointBuffer(const uint8_t& stream_id) {
    if (txBufferLocked) return nullptr;
    txBufferLocked = true;
    txBuffer[0] = session_id;
    txBuffer[1] = stream_id;
    return &(txBuffer[2]);
  }

  void returnEndpointBuffer(const uint16_t& size) {
    if (size) framer.sendFrame((const char*)(&txBuffer[0]), size + 2);
    txBufferLocked = false;
  }
};

#endif /* _SERIAL_COMMS_INTERFACE_HPP_ */
