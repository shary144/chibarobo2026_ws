#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp" // コントローラーを使うので必要

#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "robomas_interfaces/msg/can_frame.hpp"
#include "robomas_interfaces/msg/robomas_frame.hpp"

enum class home_state {
    HOME_IDLE,
    HOME_LIFT_UP,
    HOME_HOLD_TURN,
    HOME_LIFT_DOWN,
    HOME_SETUP
};

class ManualControl : public rclcpp::Node 
{
public:
    ManualControl() : Node("manual_control_node") {
        robomas_pub_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
        can_pub_ = this->create_publisher<robomas_interfaces::msg::CanFrame>("/robomas/can_tx", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 
        10, 
        std::bind(&ManualControl::joy_callback, this, std::placeholders::_1)); // /joyに情報が来たらjoy_callbackを起動するよ

        feedback_sub_ = this->create_subscription<robomas_interfaces::msg::RobomasFrame>(
        "/robomas/feedback", 
        10, 
        std::bind(&ManualControl::feedback_callback, this, std::placeholders::_1)); // feedbackの情報が来たらfeedback_callbackを起動するよ
        RCLCPP_INFO(this->get_logger(), "ManualControl起動しました。");
        }
private:
    void joy_callback(sensor_msgs::msg::Joy::SharedPtr joy) {
        if (home_state_ != home_state::HOME_IDLE) { // ホーミング中はコントローラー無効化
            homing();
            return;
        }
        float x = joy->axes[0];  // 左スティック左右
        float y = joy->axes[1];  // 左スティック上下
        float turn = joy->axes[2]; // 右スティック左右
        omuni4(x, y, turn); // 足回り

        float lift_left = joy->buttons[7]; // 十字キー上下
        float lift_right = joy->buttons[3] - joy->buttons[0]; // Yボタンで上昇、Aボタンで下降
        lift(lift_left, lift_right); //　昇降

        float hold_left = joy->buttons[4]; // LB
        float hold_right = joy->buttons[5]; // RB
        hold(hold_left, hold_right); // 把持

        float hold_turn_left = joy->axes[2]; // LT
        float hold_turn_right = joy->axes[5]; // RT
        hold_turn(hold_turn_left, hold_turn_right); // 把持回転

        int button_homing = joy->buttons[6]; // Backボタンで原点復帰
        if (button_homing) {
            homing();  
        }
    }
    robomas_interfaces::msg::RobomasFrame::SharedPtr feedback;
    void feedback_callback(robomas_interfaces::msg::RobomasFrame::SharedPtr feedback_pointer) {
        // フィードバックの処理をここに追加することができます
        // 例: フィードバックの値をログに出力するなど
        feedback = feedback_pointer;
    }
    
    home_state home_state_ = home_state::HOME_IDLE;
    bool lift_left_= false; // ここでのfalseは、ホーミングができていないということです。以下も同様です。
    bool lift_right_= false;
    bool hold_turn_left_= false;
    bool hold_turn_right_= false;

    float max_lift_left_ = 0.0;
    float max_lift_right_ = 0.0;
    float min_lift_left_ = 0.0;
    float min_lift_right_ = 0.0;
    float max_hold_turn_left_ = 0.0;
    float max_hold_turn_right_ = 0.0;
    float min_hold_turn_left_ = 0.0;
    float min_hold_turn_right_ = 0.0;  

    void homing() { // 原点復帰
        switch(home_state_) {
            case home_state::HOME_IDLE:
                home_state_ = home_state::HOME_LIFT_UP;
                RCLCPP_INFO(this->get_logger(), "Homing started.");
                lift_left_ = false;
                lift_right_ = false;
                hold_turn_left_ = false;
                hold_turn_right_ = false;
                break;
            case home_state::HOME_LIFT_UP:
                if (feedback->current[4] > 1000) lift_left_ = true; // 左昇降が完了したかどうかの判定 (現在は1000mAで終わり)
                if (feedback->current[5] > 1000) lift_right_ = true; // 右昇降が完了したかどうかの判定
                lift(!lift_left_, !lift_right_); // 昇降
                if (lift_left_ && lift_right_) {
                    max_lift_left_ = feedback->angle[4] - 1000.0; // 安全率を考慮する(1000deg)
                    max_lift_right_ = feedback->angle[5] - 1000.0; // 安全率を考慮する(1000deg)
                    home_state_ = home_state::HOME_HOLD_TURN;
                    RCLCPP_INFO(this->get_logger(), "Lift_up completed. Proceeding to hold turn.");
                    lift_left_ = false;
                    lift_right_ = false;
                }
                break;
            case home_state::HOME_HOLD_TURN:
                if (feedback->current[6] > 1000) hold_turn_left_ = true; // 左把持回転が完了したかどうかの判定
                if (feedback->current[7] > 1000) hold_turn_right_ = true; // 右把持回転が完了したかどうかの判定
                hold_turn(!hold_turn_left_, !hold_turn_right_); // 把持回転
                if (hold_turn_left_ && hold_turn_right_) {
                    home_state_ = home_state::HOME_SETUP;
                    max_hold_turn_left_ = feedback->angle[6] - 1000.0; // 安全率を考慮する(1000deg)
                    max_hold_turn_right_ = feedback->angle[7] - 1000.0; // 安全率を考慮する(1000deg)
                    min_hold_turn_left_ = max_hold_turn_left_ - 10000.0; // 水平になるくらいに調整する。
                    min_hold_turn_right_ = max_hold_turn_right_ - 10000.0; // 水平になるくらいに調整する。
                    RCLCPP_INFO(this->get_logger(), "Hold turn completed. Proceeding to setup.");
                }
                break;
            case home_state::HOME_LIFT_DOWN:
                if (feedback->current[4] > 1000) lift_left_ = true; // 左昇降が完了したかどうかの判定 (現在は1000mAで終わり)
                if (feedback->current[5] > 1000) lift_right_ = true; // 右昇降が完了したかどうかの判定
                lift(!lift_left_, !lift_right_); // 昇降
                if (lift_left_ && lift_right_) {
                    min_lift_left_ = feedback->angle[4] + 1000.0; // 安全率を考慮する(1000deg)
                    min_lift_right_ = feedback->angle[5] + 1000.0; // 安全率を考慮する(1000deg)
                    home_state_ = home_state::HOME_SETUP;
                    RCLCPP_INFO(this->get_logger(), "Lift_down completed. Proceeding to setup.");
                }
                break;
            case home_state::HOME_SETUP: {
                auto msg = robomas_interfaces::msg::RobomasPacket();
                robomas_interfaces::msg::MotorCommand cmd;
                cmd.mode = 2;

                cmd.motor_id = 5; // 昇降左
                cmd.target = min_lift_left_;
                msg.motors.push_back(cmd);

                cmd.motor_id = 6; // 昇降右
                cmd.target = min_lift_right_;
                msg.motors.push_back(cmd); 

                cmd.motor_id = 7; // 把持左
                cmd.target = min_hold_turn_left_;
                msg.motors.push_back(cmd); 

                cmd.motor_id = 8; // 把持右
                cmd.target = min_hold_turn_right_;
                msg.motors.push_back(cmd);

                robomas_pub_->publish(msg);
                
                RCLCPP_INFO(this->get_logger(), "Homing completed.");
                home_state_ = home_state::HOME_IDLE;
                break;
            }

            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown home state.");
                home_state_ = home_state::HOME_IDLE;
                break;
        }
    }
    void lift(float left, float right) { // 昇降
        auto msg = robomas_interfaces::msg::RobomasPacket();
        robomas_interfaces::msg::MotorCommand cmd;
        cmd.mode = 1;

        cmd.motor_id = 5; // 昇降左
        cmd.target = 1000.0 * left;
        msg.motors.push_back(cmd);

        cmd.motor_id = 6; // 昇降右
        cmd.target = 1000.0 * right;
        msg.motors.push_back(cmd);

        robomas_pub_->publish(msg);
    }

    bool left_running_ = false;
    bool previous_left_button_ = false;
    bool right_running_ = false;
    bool previous_right_button_ = false;
    void hold(float left, float right) { // 把持
        auto msg = robomas_interfaces::msg::CanFrame();
        if (left && !previous_left_button_) {
            left_running_ = !left_running_;
        }
        previous_left_button_ = left;

        if (right && !previous_right_button_) {
            right_running_ = !right_running_;
        }
        previous_right_button_ = right; 

        if (left_running_) {
            msg.id = 0x050;
            msg.dlc = 1;
            msg.data = {0x50};
            can_pub_->publish(msg);
        }
        if (right_running_) {
            msg.id = 0x050;
            msg.dlc = 1;
            msg.data = {0x51};
            can_pub_->publish(msg);
        }
    }

    void hold_turn(float left, float right) { // 把持回転
        auto msg = robomas_interfaces::msg::RobomasPacket();
        robomas_interfaces::msg::MotorCommand cmd;
        cmd.mode = 1;

        cmd.motor_id = 7; // 把持左
        cmd.target = min_hold_turn_left_ + (max_hold_turn_left_ - min_hold_turn_left_) * left;  
        msg.motors.push_back(cmd);

        cmd.motor_id = 8; // 把持右
        cmd.target = min_hold_turn_right_ + (max_hold_turn_right_ - min_hold_turn_right_) * right;
        msg.motors.push_back(cmd);

        robomas_pub_->publish(msg);
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

        robomas_pub_->publish(msg);
    }
// */
    rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr robomas_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;    
    rclcpp::Publisher<robomas_interfaces::msg::CanFrame>::SharedPtr can_pub_;
    rclcpp::Subscription<robomas_interfaces::msg::RobomasFrame>::SharedPtr feedback_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualControl>());
    rclcpp::shutdown();
    return 0;
}