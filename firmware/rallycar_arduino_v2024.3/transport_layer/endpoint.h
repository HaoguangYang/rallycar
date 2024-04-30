#ifndef _ENDPOINT_H_
#define _ENDPOINT_H_

#include <stdint.h>
#include <stdio.h>
#include "field_type.h"

namespace sensor_network {

class EndpointBase {
public:
    const EndpointType endpointType;
    uint8_t stream_id = (uint8_t)NodeServiceID::UNASSIGNED;

    EndpointBase(EndpointType type) : endpointType{type} {}

    virtual void spin(const uint8_t *payload, uint16_t length) {
        (void) payload;
        (void) length;
    }
};

template <typename T, typename NodeT>
class Publisher : public EndpointBase {
public:
    Publisher(NodeT* nh) : EndpointBase(EndpointType::Publisher), nh_(nh) {}

    void publish(const T& msg){
        uint8_t* outBuffer = nh_->getEndpointBuffer(this->stream_id);
        if (!outBuffer) return;
        nh_->returnEndpointBuffer(T::serialize(msg, outBuffer));
    }
protected:
    NodeT* nh_;
};

template <typename T, typename NodeT = void>
class Subscription : public EndpointBase {
public:
    typedef void (NodeT::*CallbackT) (const T& msg);

    Subscription(CallbackT callback, NodeT* nh) :
        EndpointBase{EndpointType::Subscription},
        nh_(nh),
        cb(callback)
    {}

    virtual void spin(const uint8_t *payload, uint16_t length) override final {
        static T msg;
        if (T::deserialize(msg, payload) != length) return;
        (nh_->*cb)(msg);
    }

private:
    NodeT* nh_;
    CallbackT cb;
};

template <typename T>
class Subscription<T, void> : public EndpointBase {
public:
    typedef void (*CallbackT) (const T& msg);

    Subscription(CallbackT callback) :
        EndpointBase{EndpointType::Subscription},
        cb(callback)
    {}

    virtual void spin(const uint8_t *payload, uint16_t length) override final {
        static T msg;
        if (T::deserialize(msg, payload) != length) return;
        this->cb(msg);
    }

protected:
    CallbackT cb;
};

} /* namespace sensor_network */

#endif
