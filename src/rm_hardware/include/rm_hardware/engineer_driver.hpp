#ifndef MY_ENGINEER_DRIVER_HPP
#define MY_ENGINEER_DRIVER_HPP
#include <rclcpp/rclcpp.hpp>
#include "rmctrl_msgs/msg/arm_state_data.hpp"
#include <chrono>
#include <thread>
#include <mutex>
#include <std_msgs/msg/int32.hpp>
#include "rmctrl_msgs/msg/arm_ctrl_data.hpp"
#include <cstdint>

#define JOINT_NUM 6 //关节数量


class DMDriver : public rclcpp::Node
{
public:
    // 构造函数：调用父类（rclcpp::Node）的构造函数，初始化节点
    DMDriver(const std::string &node_name = "dm_driver_node") 
        : rclcpp::Node(node_name, rclcpp::NodeOptions().use_global_arguments(true)), // 继承节点的构造
          stop_sub_(false), auto_state_mcu_(0), auto_state_nuc_(0), gripper_state_(0),arm_ctrl_mode_(0),arm_ctrl_process_state_(0)
    {
        // 初始化关节数据数组
        for (int i = 0; i < JOINT_NUM; i++) 
        {
            joint_positions_[i] = 0.0;
            joint_velocities_[i] = 0.0;
        }

        // 直接创建发布者（继承自Node，无需外部节点指针）
        pub_moveit2_arm_cmd_to_nuc_ = this->create_publisher<rmctrl_msgs::msg::ArmCtrlData>(
            "joint_cmd_from_moveit2", 
            rclcpp::QoS(10)
        );
        if (!pub_moveit2_arm_cmd_to_nuc_) 
        {
            RCLCPP_ERROR(this->get_logger(), "发布者未初始化，无法发布关节指令！");
            return;
        }

        sub_arm_state_ = this->create_subscription<rmctrl_msgs::msg::ArmStateData>(
            "/joint_state_sub_from_mcu", rclcpp::QoS(10),
            [this](const rmctrl_msgs::msg::ArmStateData::SharedPtr msg) 
            {
                std::lock_guard<std::mutex> lock(joint_mutex_);
            //在此次修正接收数据的电机转向，注意是接收
                joint_positions_[0] = -msg->joint1_position;
                joint_positions_[1] = -msg->joint2_position;
                joint_positions_[2] = -msg->joint3_position;//与下位机相反
                joint_positions_[3] = -msg->joint4_position;
                joint_positions_[4] = msg->joint5_position;
                joint_positions_[5] = msg->joint6_position;
        //-----有多个正负号的原因：不是si山代码，是达妙电机转向方向不统一，导致与坐标轴不符-----//
                joint_velocities_[0] = msg->joint1_velocity;
                joint_velocities_[1] = msg->joint2_velocity;
                joint_velocities_[2] = msg->joint3_velocity;
                joint_velocities_[3] = msg->joint4_velocity;
                joint_velocities_[4] = msg->joint5_velocity;
                joint_velocities_[5] = msg->joint6_velocity;
                
                //硬件规范写了必须是double类型，而且传回的是位置，这里用状态替代
                gripper_state_ = (msg->gripper_state > 0.5) ? -0.03 : 0.0;
                auto_state_mcu_ = msg->auto_state;
                arm_ctrl_mode_ = msg->arm_ctrl_mode;
                // RCLCPP_INFO(this->get_logger(), "回调正常触发");
            });
        sub_process_ = this->create_subscription<std_msgs::msg::Int32>(
            "arm_ctrl_process", rclcpp::QoS(10),
            [this](const std_msgs::msg::Int32::SharedPtr msg) 
            {
                std::lock_guard<std::mutex> lock(joint_mutex_);
                arm_ctrl_process_state_ = static_cast<int8_t>(msg->data);
                RCLCPP_DEBUG(this->get_logger(), "arm_ctrl_process_state 更新为 %d", arm_ctrl_process_state_);
            });
        RCLCPP_INFO(this->get_logger(), "DMDriver 初始化完成，订阅已创建");
    }

    // 析构函数：自动调用父类析构，无需手动释放节点
    ~DMDriver() override 
    {
        stop_sub_ = true;
        // 若使用了自定义线程，等待退出（可选，推荐用ROS2内置回调）
        if (sub_thread_.joinable()) 
        {
            sub_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "DMDriver驱动已销毁");
    }

    bool check_joint_state_topic_data(double timeout_sec = 1.0)
    {
        if (!rclcpp::ok()) 
        {
            RCLCPP_ERROR(this->get_logger(), "ROS2没有初始化");
            return false;
        }

        bool has_data = false;
        // 直接使用自身节点创建临时订阅器
        auto sub = this->create_subscription<rmctrl_msgs::msg::ArmStateData>(
            "/joint_state_sub_from_mcu",
            rclcpp::QoS(10),
            [&has_data](const rmctrl_msgs::msg::ArmStateData::SharedPtr /*msg*/) { 
                has_data = true; 
            }
        );

        const auto start_time = this->now();
        while (rclcpp::ok()) 
        {
            const double elapsed = (this->now() - start_time).seconds();
            if (elapsed >= timeout_sec) 
            {
                RCLCPP_ERROR(this->get_logger(), "话题检查超时 (%.2fs)", elapsed);
                break;
            }
            rclcpp::spin_some(this->get_node_base_interface()); // 继承的节点接口
            if (has_data) 
            {
                RCLCPP_INFO(this->get_logger(), "接收到下位机发来的机械臂数据");
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        sub.reset();
        return has_data;
    }

    std::vector<double> getPosition(void)
    {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        std::vector<double> pos_array(JOINT_NUM);
        for (int i = 0; i < JOINT_NUM; i++) 
        {
            pos_array[i] = joint_positions_[i];
        }
        return pos_array;
    }
    
    void setTargetPositionRadian(double joint_position_cmd_array[6], int8_t gripper_ctrl) 
    {
        if (!pub_moveit2_arm_cmd_to_nuc_) 
        {
            RCLCPP_ERROR(this->get_logger(), "发布者未初始化，无法发送控制指令！");
            return;
        }

        rmctrl_msgs::msg::ArmCtrlData msg;
        //在此次修正发送命令的电机转向，注意发送
        msg.joint1_position = joint_position_cmd_array[0];
        msg.joint2_position = -joint_position_cmd_array[1];
        msg.joint3_position = -joint_position_cmd_array[2];//与下位机相反
        msg.joint4_position = joint_position_cmd_array[3];
        msg.joint5_position = joint_position_cmd_array[4];
        msg.joint6_position = -joint_position_cmd_array[5];
        msg.gripper_ctrl = gripper_ctrl;
        msg.auto_state = 1;
        msg.arm_ctrl_process_state = arm_ctrl_process_state_;

        pub_moveit2_arm_cmd_to_nuc_->publish(msg);
    }

    double getGripperState(void) 
    {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        return gripper_state_;
    }
    
    int getActiveCmd(void) 
    {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        RCLCPP_INFO(this->get_logger(), "auto_state_mcu_=%d",auto_state_mcu_);
        return auto_state_mcu_;
    }

    void set_auto_state_nuc(int set_state) 
    {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        if (set_state == 1) {
            auto_state_nuc_ = 1;
            RCLCPP_INFO(this->get_logger(), "自动状态已设置为：开启");
        } else if (set_state == 0) {
            auto_state_nuc_ = 0;
            RCLCPP_INFO(this->get_logger(), "自动状态已设置为：关闭");
        } else {
            RCLCPP_ERROR(this->get_logger(), "设置auto_state_nuc的值不合法!仅支持0或1");
        }
    }

private:
    std::thread sub_thread_;
    bool stop_sub_ = false;
    rclcpp::Publisher<rmctrl_msgs::msg::ArmCtrlData>::SharedPtr pub_moveit2_arm_cmd_to_nuc_;
    rclcpp::Subscription<rmctrl_msgs::msg::ArmStateData>::SharedPtr sub_arm_state_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_process_;
    int8_t auto_state_mcu_;       
    int8_t auto_state_nuc_;       
    double gripper_state_;   
    int8_t arm_ctrl_mode_;
    int8_t arm_ctrl_process_state_;         
    double joint_positions_[JOINT_NUM];  
    double joint_velocities_[JOINT_NUM]; 
    std::mutex joint_mutex_;      

};

#endif