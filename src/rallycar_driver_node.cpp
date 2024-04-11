#include "rallycar/serial_comms_interface.hpp"
#include "../firmware/rallycar_arduino_v2024.3/rallycar_msgs/msg/imu_raw.h"
#include "../firmware/rallycar_arduino_v2024.3/std_msgs/msg/float32.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>

namespace rallycar {

class RallycarDriverNode : public rclcpp::Node, public SerialCommsInterface<256, 256, 3> {
    using SerialImuRaw = rallycar_msgs__msg::ImuRaw;
    using SerialFloat32 = std_msgs__msg::Float32;
    using Imu = sensor_msgs::msg::Imu;
    using Float32 = std_msgs::msg::Float32;
public:
    RallycarDriverNode(
        const rclcpp::NodeOptions& options,
        const uint8_t& session_id = 0
    ) :
        rclcpp::Node("rallycar_driver_node", options),
        SerialCommsInterface(session_id),
        ser_imu_sub(&RallycarDriverNode::pub_ros_imu, this),
        ser_accel_brake_pub(this),
        ser_steer_pub(this)
    {
        this->setPort("/dev/ttyACM0");
        this->setBaudrate(115200);
        this->transferInit();
        ros_imu_pub = this->create_publisher<Imu>("/imu", rclcpp::SensorDataQoS());
        ros_acc_sub = this->create_subscription<Float32>(
            "/accelerator_cmd", rclcpp::SensorDataQoS(),
            std::bind(&RallycarDriverNode::pub_ser_acc, this, std::placeholders::_1));
        ros_steer_sub = this->create_subscription<Float32>(
            "/steering_cmd", rclcpp::SensorDataQoS(),
            std::bind(&RallycarDriverNode::pub_ser_steer, this, std::placeholders::_1));
        registerEndpoint(&ser_imu_sub, 0);
        registerEndpoint(&ser_accel_brake_pub, 1);
        registerEndpoint(&ser_steer_pub, 2);
        serial_rx_timer = this->create_wall_timer(std::chrono::milliseconds(5),
            std::bind(&RallycarDriverNode::spin, this));
    }

    void pub_ros_imu(const SerialImuRaw& msg) {
        imu.header.stamp.sec = msg.stamp.sec;
        imu.header.stamp.nanosec = msg.stamp.nanosec;

        imu.orientation.x = msg.orientation.x;
        imu.orientation.y = msg.orientation.y;
        imu.orientation.z = msg.orientation.z;
        imu.orientation.w = msg.orientation.w;

        imu.angular_velocity.x = msg.angular_velocity.x;
        imu.angular_velocity.y = msg.angular_velocity.y;
        imu.angular_velocity.z = msg.angular_velocity.z;

        imu.linear_acceleration.x = msg.linear_acceleration.x;
        imu.linear_acceleration.y = msg.linear_acceleration.y;
        imu.linear_acceleration.z = msg.linear_acceleration.z;

        ros_imu_pub->publish(imu);
    }

    void pub_ser_acc(const Float32& msg) {
        ser_accel_cmd.data = msg.data;
        ser_accel_brake_pub.publish(ser_accel_cmd);
    }

    void pub_ser_steer(const Float32& msg) {
        ser_steer_cmd.data = msg.data;
        ser_steer_pub.publish(ser_steer_cmd);
    }

protected:
    Imu imu;
    SerialFloat32 ser_accel_cmd, ser_steer_cmd;

    sensor_network::Subscription<SerialImuRaw, RallycarDriverNode> ser_imu_sub;
    sensor_network::Publisher<SerialFloat32, RallycarDriverNode> ser_accel_brake_pub, ser_steer_pub;

    rclcpp::TimerBase::SharedPtr serial_rx_timer;
    rclcpp::Publisher<Imu>::SharedPtr ros_imu_pub;
    rclcpp::Subscription<Float32>::SharedPtr ros_acc_sub, ros_steer_sub;

    virtual SerialTime time_now() override final {
        SerialTime ret;
        double t = now().seconds();
        ret.sec = (int32_t)t;
        ret.nanosec = (uint32_t)((t - (double)ret.sec) * 1000000000U);
        return ret;
    }
};

}   // namespace rallycar

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rallycar::RallycarDriverNode)
