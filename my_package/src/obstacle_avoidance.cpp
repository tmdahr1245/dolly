#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class ObstacleAvoidance: public rclcpp::Node{
    public:
        ObstacleAvoidance(): Node("ObstacleAvoidance"){
            auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "laser_scan", default_qos,
                std::bind(&ObstacleAvoidance::topic_callback, this, _1));
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        }
    private:
        void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg){
            float min = 10;
            for(int i = 0; i < 200; i++){
                float current = _msg->ranges[i];
                if(current < min){
                    min = current;
                }
            }
            auto message = this->calculateVelMsg(min);
            publisher_->publish(message);
        }
        geometry_msgs::msg::Twist calculateVelMsg(float distance){
            auto msg = geometry_msgs::msg::Twist();
            RCLCPP_INFO(this->get_logger(), "Distance is: '%f'", distance);
            if(distance < 1){
                msg.linear.x = 0;
                msg.angular.z = 0.3;
            }
            else{
                msg.linear.x = 0.3;
                msg.angular.z = 0;
            }
            return msg;
        }
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidance>());
    rclcpp::shutdown();

    return 0;
}