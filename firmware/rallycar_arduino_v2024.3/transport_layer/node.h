#ifndef _NODE_H_
#define _NODE_H_

#include "../builtin_interfaces/msg/time.h"
#include "clock.h"
#include "endpoint.h"
#include "field_type.h"

namespace sensor_network {
/* Base class for multiple nodes */
class NodeBase {
public:
    const uint8_t session_id;

    NodeBase(uint8_t session = 0) : session_id(session) {}

    virtual void onIncomingData(const uint8_t* data, const uint16_t& length) = 0;
};

/* Node class where customized node can inherit from */
template<class Hardware,
         class Framer,
         uint8_t MAX_TIMERS = 25,
         uint8_t MAX_ENDPOINTS = 25,
         uint16_t TX_MTU = 512>
class Node_ : public NodeBase {
private:
    uint8_t txBuffer[TX_MTU];
    bool txBufferLocked = false;
    Hardware* hw_;
    Framer* framer;
    WallClock<Hardware> clock;
    WallTimerBase* timers[MAX_TIMERS] = {nullptr};
    uint8_t timerCount = 0;
    EndpointBase* endpoints[MAX_ENDPOINTS] = {nullptr};
    uint8_t max_stream_record = 0;

protected:
    NodeState state = NodeState::OUT_OF_SYNC;

    void reset() {
        state = NodeState::OUT_OF_SYNC;
        onReset();
    }

    bool clock_sync() {
        returnEndpointBuffer(
            clock.spin(getEndpointBuffer((uint8_t)NodeServiceID::TIME_SYNC_ID))
        );
        return clock.is_synced();
    }

public:
    typedef NodeBase BaseT;
    typedef WallClock<Hardware> ClockT;

    Node_(
        Hardware* hw,
        Framer* fr,
        uint8_t session_id = 0,
        uint8_t clock_sync_interval_sec = 5,
        uint8_t clock_sync_timeout_sec = 3) :
        NodeBase{session_id},
        hw_(hw),
        framer(fr),
        clock{hw_, clock_sync_interval_sec, clock_sync_timeout_sec}
    {}

    uint8_t* getEndpointBuffer(const uint8_t& stream_id) {
        if (txBufferLocked) return nullptr;
        txBufferLocked = true;
        txBuffer[0] = session_id;
        txBuffer[1] = stream_id;
        return &(txBuffer[2]);
    }

    void returnEndpointBuffer(const uint16_t& size) {
        if (size)
            framer->sendFrame((const char*)txBuffer, size+2);
        txBufferLocked = false;
    }

    WallClock<Hardware>* get_clock() const { return &clock; }

    void spin() {
        // this function is strictly single-threaded, which guarantees
        // txBuffer is only used by one function call at a time.
        int data = hw_->read();
        while (data >= 0) {
            this->framer->receiveChar((uint8_t)data);
            data = hw_->read();
        }
        if (state == NodeState::RUNNING) {
            for (uint8_t n = 0; n < timerCount; n++) {
                timers[n]->spin();
            }
        }
        if (clock_sync())
            state = NodeState::RUNNING;
        else {
            state = NodeState::OUT_OF_SYNC;
            onReset();
        }
    }

    bool registerEndpoint(EndpointBase* endpoint, const uint8_t& stream_id) {
        if (stream_id >= MAX_ENDPOINTS) return false;
        if (endpoints[stream_id] != nullptr) return false;
        endpoints[stream_id] = endpoint;
        endpoints[stream_id]->stream_id = stream_id;
        if (stream_id >= max_stream_record) max_stream_record = stream_id + 1;
        return true;
    }

    bool registerTimer(WallTimerBase* timer) {
        if (timerCount >= MAX_TIMERS) return false;
        timers[timerCount ++] = timer;
        return true;
    }

    virtual void onIncomingData(const uint8_t* data, const uint16_t& length) override final {
        switch (data[0]) {
            case (uint8_t)NodeServiceID::TIME_SYNC_ID:
                clock.onSyncReturn(&data[1], length-1);
                break;
            case (uint8_t)NodeServiceID::RESET_ID:
                reset();
                break;
            default:
                if (state != NodeState::RUNNING || data[0] >= max_stream_record) return;
                break;
        }
        EndpointBase* e = endpoints[data[0]];
        if (!e) return;
        if (e->endpointType != EndpointType::Subscription) return;
        //if (!(e->initialized)) return;
        e -> spin(&(data[1]), length-1);
    }

    virtual void onReset() {}

    ~Node_() = default;

};


void frame_dispatcher(uint8_t* buf, uint16_t sz, NodeBase** nodes, uint8_t nodes_count = 1) {
#if 0
    if (buf[0] >= nodes_count) return;
    NodeBase* n = nodes[buf[0]];
    if (!n) return;
    if (buf[0] != n->session_id) return;
    n->onIncomingData(&(buf[1]), sz-1);
#else
    if (buf[0] != nodes[0]->session_id) return;
    nodes[0]->onIncomingData(&(buf[1]), sz-1);
#endif
}


template <typename HardwareT, uint8_t MAX_NODES = 1>
class CommsInterface {
  protected:
    HardwareT* hw_;
    NodeBase* nodes[MAX_NODES] = {nullptr};

  public:
    CommsInterface(HardwareT* hw) : hw_(hw) {}

    bool register_node(NodeBase* n){
      // conflicting session id
      if (nodes[n->session_id]) return false;
      nodes[n->session_id] = n;
    }

    void send_char(const uint8_t& data) {
        hw_->write(&data, 1);
    }

    void dispatch_frame(const uint8_t* buf, const uint16_t& sz) {
        frame_dispatcher(buf, sz, &(nodes[0]), MAX_NODES);
    }
};

}   /* namespace sensor_network */

#endif
