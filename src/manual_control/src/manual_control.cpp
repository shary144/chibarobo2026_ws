#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp" // コントローラーを使うので必要

#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"

class ManualControl : public rclcpp::Node 
{
public:
    ManualControl() : Node("manual_control_node") {
        pub_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 
        10, 
        std::bind(&ManualControl::joy_callback, this, std::placeholders::_1)); // /joyに情報が来たらjoy_callbackを起動するよ
        RCLCPP_INFO(this->get_logger(), "ManualControl起動しました。");
        }
private:
    void joy_callback(sensor_msgs::msg::Joy::SharedPtr joy) {

        float x = joy->axes[0];  // 左スティック左右
        float y = joy->axes[1];  // 左スティック上下
        omuni4(x, y); // 足回り
        
        // 
        auto msg = robomas_interfaces::msg::RobomasPacket();
        robomas_interfaces::msg::MotorCommad cmd;
        cmd.mode = 1;
        cmd.motor_id = 5;
        cmd.target = 
    }

    void lift(int left, int right) {

    }

    void 

    void omuni4(float x, float y) { // オムニ4輪用
        float speed_max = 1000.0;

        auto msg = robomas_interfaces::msg::RobomasPacket();
        robomas_interfaces::msg::MotorCommand cmd;
        cmd.mode = 1;

        cmd.motor_id = 1; // 右上
        cmd.target = speed_max * (-0.707 * x + 0.707 * y);
        msg.motors.push_back(cmd);

        cmd.motor_id = 2; // 左上
        cmd.target = speed_max * (-0.707 * x + -0.707 * y);
        msg.motors.push_back(cmd);

        cmd.motor_id = 3; // 左下
        cmd.target = speed_max * (0.707 * x + -0.707 * y);
        msg.motors.push_back(cmd);
        
        cmd.motor_id = 4; // 右下
        cmd.target = speed_max * (0.707 * x + 0.707 * y);
        msg.motors.push_back(cmd);

        pub_->publish(msg);
    }
// */
    rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualControl>());
    rclcpp::shutdown();
    return 0;
}