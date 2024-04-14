#include "rallycar/serial_comms_interface.hpp"
#include "../firmware/rallycar_arduino_v2024.3/rallycar_msgs/msg/imu_raw.h"
#include "../firmware/rallycar_arduino_v2024.3/std_msgs/msg/float32.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <unistd.h>

namespace rallycar {

class RallycarDriverNode : public SerialCommsInterface<256, 256, 3> {
    using SerialImuRaw = rallycar_msgs__msg::ImuRaw;
    using SerialFloat32 = std_msgs__msg::Float32;
    using Imu = sensor_msgs::Imu;
    using Float32 = std_msgs::Float32;
public:
    RallycarDriverNode(
        ros::NodeHandle* nh,
        const uint8_t& session_id = 0
    ) :
        nh_(nh),
        SerialCommsInterface(session_id),
        ser_imu_sub(&RallycarDriverNode::pub_ros_imu, this),
        ser_accel_brake_pub(this),
        ser_steer_pub(this)
    {
        // set node config at startup from parameters
        std::string port;
        ros::param::param<std::string>("serial_port_fd", port, "/dev/ttyACM0");
        this->setPort(port);
        int baud;
        ros::param::param<int>("serial_port_baudrate", baud, 115200);
        this->setBaudrate((unsigned long)baud);
        std::string imu_frame;
        ros::param::param<std::string>("imu_frame_id", imu_frame, "imu_frame");
        imu.header.frame_id = imu_frame;

        // initialize serial port and publishers & subscribers
        while (ros::ok()){
            if (this->transferInit()) break;
            sleep(1);
        }
        // ros side
        ros_imu_pub = nh_->advertise<Imu>("/imu", 1);
        ros_acc_sub = nh_->subscribe("/accelerator_cmd", 1, &RallycarDriverNode::pub_ser_acc, this);
        ros_steer_sub = nh_->subscribe("/steering_cmd", 1, &RallycarDriverNode::pub_ser_steer, this);

        // serial port side, make sure the IDs are set the same as defined in the firmware code
        registerEndpoint(&ser_imu_sub, 0);
        registerEndpoint(&ser_accel_brake_pub, 1);
        registerEndpoint(&ser_steer_pub, 2);
    }

    void run(){
        ros::Rate loop_rate(200);
        while (ros::ok()){
            this->spin();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void pub_ros_imu(const SerialImuRaw& msg) {
        // upon reception of latest IMU data from serial port, deserialize it
        // and bridge it to the ROS side
        imu.header.stamp.sec = msg.stamp.sec;
        imu.header.stamp.nsec = msg.stamp.nanosec;

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

        ros_imu_pub.publish(imu);
    }

    void pub_ser_acc(const Float32& msg) {
        // upon reception of ROS side data, serialize it and bridge it
        // to the serial port side.
        ser_accel_cmd.data = msg.data;
        ser_accel_brake_pub.publish(ser_accel_cmd);
    }

    void pub_ser_steer(const Float32& msg) {
        // upon reception of ROS side data, serialize it and bridge it
        // to the serial port side.
        ser_steer_cmd.data = msg.data;
        ser_steer_pub.publish(ser_steer_cmd);
    }

protected:
    ros::NodeHandle* nh_;
    Imu imu;
    SerialFloat32 ser_accel_cmd, ser_steer_cmd;

    sensor_network::Subscription<SerialImuRaw, RallycarDriverNode> ser_imu_sub;
    sensor_network::Publisher<SerialFloat32, RallycarDriverNode> ser_accel_brake_pub, ser_steer_pub;

    ros::Publisher ros_imu_pub;
    ros::Subscriber ros_acc_sub, ros_steer_sub;

    virtual SerialTime time_now() override final {
        // time getter for time synchoronization with serial devices
        SerialTime ret;
        double t = ros::Time::now().toSec();
        ret.sec = (int32_t)t;
        ret.nanosec = (uint32_t)((t - (double)ret.sec) * 1000000000U);
        return ret;
    }
};

}   // namespace rallycar

int main(int argc, char **argv) {
    ros::init(argc, argv, "rallycar_driver_node");
    ros::NodeHandle n;
    rallycar::RallycarDriverNode dri(&n);
    dri.run();
    return 0;
}
