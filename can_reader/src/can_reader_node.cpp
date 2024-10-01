#include "rclcpp/rclcpp.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <thread>
#include <functional>
#include <cmath>
#include <bitset>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace drivers::socketcan;

class CanReceiverNode : public rclcpp::Node
{
public:
    CanReceiverNode()
        : Node("can_receiver_node"), receiver_("can0", false), speed_car(0.0), steering_angle_not_norm(0.0), steering_angle_norm(0.0), x(0.0), y(0.0), th(0.0), wheelbase(2.5) // Assuming a wheelbase of 2.5 meters
    {
        // Create a publisher for CAN data (for debugging purposes)
        publisher_ = this->create_publisher<std_msgs::msg::String>("can_data", 10);

        // Create a publisher for odometry
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // Create the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Start the thread for listening to CAN messages
        listener_thread_ = std::thread(std::bind(&CanReceiverNode::listen_can, this));

        // Start the odometry computation timer, running every 100ms (0.1s)
        odom_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CanReceiverNode::compute_odometry, this));
    }

    ~CanReceiverNode()
    {
        if (listener_thread_.joinable())
        {
            listener_thread_.join();
        }
    }

private:
    void listen_can()
    {
        uint8_t data[8]; // CAN frame max length is 8 bytes
        while (rclcpp::ok())
        {
            try
            {
                // Block and wait for a CAN message to be received
                auto can_frame = receiver_.receive(data, std::chrono::milliseconds(100)); // Wait with a 100ms timeout

                // Process the received message in a callback-style manner
                handle_can_message(can_frame, data);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error reading CAN data: %s", e.what());
            }
        }
    }

    double degrees_to_radians(double degrees) const
    {
        return degrees * (M_PI / 180.0); // M_PI is defined in <cmath>
    }

    // Callback function to handle CAN messages
    void handle_can_message(const CanId &can_frame, const uint8_t *data)
    {
        if (can_frame.identifier() == 1285)
        {
            // Handle speed data
            std::string binary_list[8];
            for (int i = 0; i < 8; ++i)
            {
                binary_list[i] = std::bitset<8>(data[i]).to_string();
            }

            // Combine bytes 2 and 3
            std::string combined_binary = binary_list[2] + binary_list[3];
            int combined_integer = std::stoi(combined_binary, nullptr, 2);

            // Handle negative values (two's complement for 16-bit)
            if (combined_integer & (1 << 15))
            {
                combined_integer = combined_integer - (1 << 16);
            }

            speed_car = combined_integer * 0.001;

            RCLCPP_INFO(this->get_logger(), "Received CAN frame: ID=1285, Speed=%.3f", speed_car);
        }

        if (can_frame.identifier() == 1282)
        {
            // Handle steering data
            std::string binary_list[8];
            for (int i = 0; i < 8; ++i)
            {
                binary_list[i] = std::bitset<8>(data[i]).to_string();
            }

            // Combine bytes 3 and 4
            std::string combined_binary = binary_list[3] + binary_list[4];
            int combined_integer = std::stoi(combined_binary, nullptr, 2);

            double real_value = combined_integer * 1.0; // Raw integer value

            steering_angle_not_norm = real_value - 500.0;
            steering_angle_norm = degrees_to_radians((steering_angle_not_norm + 500) * 0.06 - 30);

            RCLCPP_INFO(this->get_logger(), "Received CAN frame: ID=1282, Steering Angle Norm=%.3f", steering_angle_norm);
        }
    }

    void compute_odometry()
    {
        // Assuming a constant time interval of 0.1 seconds
        double dt = 0.1;
        double delta_x = speed_car * cos(th) * dt;
        double delta_y = speed_car * sin(th) * dt;
        double delta_th = speed_car * tan(steering_angle_norm) / wheelbase * dt;

        // Update the position and orientation
        x += delta_x;
        y += delta_y;
        th += delta_th;

        // Normalize the orientation angle to keep it within the range of -pi to pi
        th = atan2(sin(th), cos(th));

        // Get the current time
        auto current_time = this->now();

        // Create the odometry message
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // Set position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.orientation.z = sin(th / 2.0);
        odom.pose.pose.orientation.w = cos(th / 2.0);

        // Publish the odometry message
        odom_pub_->publish(odom);

        // Create the transform message
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.rotation.z = sin(th / 2.0);
        transform.transform.rotation.w = cos(th / 2.0);

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform);
    }

    SocketCanReceiver receiver_; // CAN receiver
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::thread listener_thread_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    // Processed data values
    double speed_car;
    double steering_angle_not_norm;
    double steering_angle_norm;

    // Odometry state
    double x, y, th;  // Position and orientation
    double wheelbase; // Distance between the front and rear axles (in meters)
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanReceiverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
