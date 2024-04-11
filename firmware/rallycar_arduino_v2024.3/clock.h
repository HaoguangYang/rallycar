#ifndef _CLOCK_H_
#define _CLOCK_H_

#include <stdint.h>
#include "builtin_interfaces/msg/time.h"

namespace time_rounding {
    template<typename T>
    T secToUSec(const T& sec) {
        return sec * T(1000000);
    }

    template<typename T>
    T uSecToSec(const T& us) {
        return us / T(1000000);
    }

    template<typename T>
    T secToMSec(const T& sec) {
        return sec * T(1000);
    }

    template<typename T>
    T mSecToSec(const T& ms) {
        return ms / T(1000);
    }

    // fast version with 5% error
    uint32_t secToUSecFast(const uint32_t& sec) {
        return sec << 20;
    }

    uint32_t uSecToSecFast(const uint32_t& us) {
        return us >> 20;
    }

    // fast version with 2.5% error
    uint32_t secToMSecFast(const uint32_t& sec) {
        return sec << 10;
    }

    uint32_t mSecToSecFast(const uint32_t& ms) {
        return ms >> 10;
    }
}

namespace sensor_network {

template <class Hardware>
class WallClock {
public:
    WallClock(Hardware* hw, uint8_t sync_interval_sec = 5, uint8_t sync_timeout_sec = 3) :
        hw_(hw),
        last_sync_micros(0),
        time_base(),
        sync_interval(sync_interval_sec),
        sync_timeout(sync_timeout_sec)
    {}

    void onSyncReturn(uint8_t* payload, uint16_t length) {
        // invalid situations -- no initialization initiated yet
        if ((!last_sync_micros) && (!last_sync_successful)) return;
        // header already sorted and removed
        uint32_t diff = hw_->time() - last_sync_micros;
        uint8_t staleness = (uint8_t)time_rounding::uSecToSecFast(diff);
        // response is too late
        if (staleness >= sync_timeout) return;
        uint8_t* ptr = payload;
        // extract repeater
        builtin_interfaces__msg::Time t;
        uint16_t len1 = builtin_interfaces__msg::Time::deserialize(t, ptr);
        // repeater is incorrect
        if (builtin_interfaces__msg::Time::toMicroSec(t) != last_sync_micros)
            return;
        // data corrupted
        if (length != builtin_interfaces__msg::Time::deserialize(t, ptr+len1) + len1)
            return;
        // all good
        time_base = t;
        // mechanism of NTP
        last_sync_micros += diff/2;
        last_sync_successful = last_sync_micros;
        synced = true;
    }

    uint16_t spin(uint8_t* txBuffer) {
        uint8_t staleness = (uint8_t)time_rounding::uSecToSecFast(
            hw_->time() - last_sync_micros);
        uint16_t ret = 0;
        if (staleness >= sync_interval || (!synced && !last_sync_micros && !last_sync_successful))
            ret = syncInit(txBuffer);
        return ret;
    }

    bool is_synced() {
        uint8_t staleness = (uint8_t)time_rounding::uSecToSecFast(
            hw_->time() - last_sync_successful);
        if (staleness >= 5 * sync_interval + sync_timeout) synced = false;
        return synced;
    }

    builtin_interfaces__msg::Time now() {
        uint32_t diff_us = hw_->time() - last_sync_successful;
        return builtin_interfaces__msg::Time(
            time_base.sec + diff_us/1000000UL,
            time_base.nanosec + (diff_us % 1000000UL) * 1000
        );
    }

    uint32_t raw_value_us() {
        return hw_->time();
    }

protected:
    Hardware* hw_;
    uint32_t last_sync_micros;
    builtin_interfaces__msg::Time time_base;
    uint8_t sync_interval, sync_timeout;
    bool synced = false;
    uint32_t last_sync_successful = 0;

    uint16_t syncInit(uint8_t* txBuffer) {
        uint8_t* ptr = txBuffer;
        last_sync_micros = hw_->time();
        time_base = builtin_interfaces__msg::Time::fromMicroSec(last_sync_micros);
        ptr += builtin_interfaces__msg::Time::serialize(time_base, ptr);
        return (ptr - txBuffer);
    }
};

class WallTimerBase {
public:
    WallTimerBase(const uint32_t& period_us) : period(period_us), last_exec(0) {}
    virtual void spin() = 0;
    virtual void reset() = 0;

protected:
    uint32_t period;
    uint32_t last_exec;
};

template <class ClockT, class NodeT = void>
class WallTimer : public WallTimerBase {
public:
    typedef void (NodeT::*CallbackT) (void);

    WallTimer(ClockT* clk, const uint32_t& period_us, CallbackT callback, NodeT* nh) :
        WallTimerBase(period_us),
        clk_(clk),
        nh_(nh),
        cb(callback)
    {}

    virtual void spin() override final {
        if (uint32_t now = clk_->raw_value_us() >= (last_exec + period)) {
            last_exec = now;
            (nh_->*cb)();
        }
    }

    void reset() override final {
        last_exec = clk_->raw_value_us();
    }

protected:
    ClockT* clk_;
    NodeT* nh_;
    CallbackT cb;
};

template <class ClockT>
class WallTimer<ClockT, void> : public WallTimerBase {
public:
    typedef void (*CallbackT) (void);

    WallTimer(ClockT* clk, const uint32_t& period_us, CallbackT callback) :
        WallTimerBase(period_us),
        clk_(clk),
        cb(callback)
    {}

    virtual void spin() override final {
        if (uint32_t now = clk_->raw_value_us() >= (last_exec + period)) {
            last_exec = now;
            this->cb();
        }
    }

    virtual void reset() override final {
        last_exec = clk_->raw_value_us();
    }

protected:
    ClockT* clk_;
    CallbackT cb;
};

} /* namespace sensor_network */
#endif
