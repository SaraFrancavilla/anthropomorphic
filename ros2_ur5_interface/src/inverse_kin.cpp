#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>

#include "ur5_globals.hpp"

// Classe del nodo
class Inverse_kin : public rclcpp::Node
{
public:
    Inverse_kin() : Node("inverse_kin")
    {
        // Creazione di una subscription per ricevere la posizione dell'end effector
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "end_effector_position", 10, std::bind(&Inverse_kin::end_effector_callback, this, std::placeholders::_1));

        // Creazione di un publisher per inviare gli angoli delle giunture
        joint_angles_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_angles", 10);
    }

private:
    void end_effector_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // Log della posizione dell'end effector ricevuta
        RCLCPP_INFO(this->get_logger(), "Received end effector position: x=%.2f, y=%.2f, z=%.2f",
                    msg->position.x, msg->position.y, msg->position.z);

        // Definire un set di angoli di giuntura predefiniti
        std_msgs::msg::Float64MultiArray joint_angles_msg;
        joint_angles_msg.data = {-1.41, -0.96, -1.8, -1.96, -1.60, 0.0}; // Angoli esempio

        // Pubblicare gli angoli delle giunture
        joint_angles_publisher_->publish(joint_angles_msg);

        RCLCPP_INFO(this->get_logger(), "Published predefined joint angles.");
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_angles_publisher_;
};

// Funzione main per eseguire il nodo
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // Inizializzare rclcpp

    // Creazione di un'istanza del nodo
    rclcpp::spin(std::make_shared<Inverse_kin>()); // Esegui il nodo

    rclcpp::shutdown(); // Terminare rclcpp
    return 0;
}
