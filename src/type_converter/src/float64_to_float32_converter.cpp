#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"

class FloatConverter : public rclcpp::Node {
public:
    FloatConverter() : Node("float_converter") {
        // 订阅 Float64 话题
        sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/tita4264886/robot_vel", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                // 转换为 Float32
                auto float32_msg = std_msgs::msg::Float32();
                float32_msg.data = static_cast<float>(msg->data);
                
                // 发布 Float32 话题
                pub_->publish(float32_msg);
                
                // RCLCPP_INFO(this->get_logger(), 
                //     "Converted: %.6f (64-bit) -> %.6f (32-bit)",
                //     msg->data, float32_msg.data);
            });

        // 发布 Float32 话题
        pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/tita4264886/robot_vel_32", 10);

        // RCLCPP_INFO(this->get_logger(), "Converter ready!");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloatConverter>());
    rclcpp::shutdown();
    return 0;
}