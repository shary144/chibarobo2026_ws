#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp" // コントローラーを使うので必要

#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "robomas_interfaces/msg/can_frame.hpp"

class ManualControl : public rclcpp::Node 
{
public:
    ManualControl() : Node("manual_control_node") {
        pub_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
        can_pub_ = this->create_publisher<robomas_interfaces::msg::CanFrame>("/robomas/can_tx", 10);
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
        float turn = joy->axes[2]; // 右スティック左右
        omuni4(x, y, turn); // 足回り

        float lift_left = joy->buttons[7]; // 十字キー上下
        float lift_right = joy->buttons[3] - joy->buttons[0]; // Yボタンで上昇、Aボタンで下降
        lift(lift_left, lift_right); //　昇降

        float hold_left = joy->buttons[4]; // LB
        float hold_right = joy->buttons[5]; // RB
        hold(hold_left, hold_right); // 把持回転

        float hold_turn_left = joy->buttons[6]; // LT
        float hold_turn_right = joy->buttons[7]; // RT
        hold_turn(hold_turn_left, hold_turn_right); // 把持回転
    }

    void lift(int left, int right) {
        auto msg = robomas_interfaces::msg::RobomasPacket();
        robomas_interfaces::msg::MotorCommand cmd;
        cmd.mode = 1;

        cmd.motor_id = 5; // 昇降左
        cmd.target = 1000.0 * left;
        msg.motors.push_back(cmd);

        cmd.motor_id = 6; // 昇降右
        cmd.target = 1000.0 * right;
        msg.motors.push_back(cmd);

        pub_->publish(msg);
    }

    void hold(int left, int right) {
        auto msg = robomas_interfaces::msg::CanFrame();
        if (left) {
            msg.id = 0x050;
            msg.dlc = 1;
            msg.data = {0x50};
            can_pub_->publish(msg);
        }
        if (right) {
            msg.id = 0x050;
            msg.dlc = 1;
            msg.data = {0x51};
            can_pub_->publish(msg);
        }
    }

    void hold_turn(int left, int right) {
        auto msg = robomas_interfaces::msg::RobomasPacket();
        robomas_interfaces::msg::MotorCommand cmd;
        cmd.mode = 1;

        cmd.motor_id = 7; // 把持左
        cmd.target = 1000.0 * left;  
        msg.motors.push_back(cmd);

        cmd.motor_id = 8; // 把持右
        cmd.target = 1000.0 * right;
        msg.motors.push_back(cmd);

        pub_->publish(msg);
    }

    void omuni4(float x, float y, float turn) { // オムニ4輪用
        float speed_max = 1000.0;

        auto msg = robomas_interfaces::msg::RobomasPacket();
        robomas_interfaces::msg::MotorCommand cmd;
        cmd.mode = 1;

        cmd.motor_id = 1; // 右上
        cmd.target = speed_max * (-0.707 * x + 0.707 * y + turn); 
        msg.motors.push_back(cmd);

        cmd.motor_id = 2; // 左上
        cmd.target = speed_max * (-0.707 * x + -0.707 * y + turn);
        msg.motors.push_back(cmd);

        cmd.motor_id = 3; // 左下
        cmd.target = speed_max * (0.707 * x + -0.707 * y + turn);
        msg.motors.push_back(cmd);
        
        cmd.motor_id = 4; // 右下
        cmd.target = speed_max * (0.707 * x + 0.707 * y + turn);
        msg.motors.push_back(cmd);

        pub_->publish(msg);
    }
// */
    rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;    
    rclcpp::Publisher<robomas_interfaces::msg::CanFrame>::SharedPtr can_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualControl>());
    rclcpp::shutdown();
    return 0;
}