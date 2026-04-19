#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <rmctrl_msgs/msg/pose_command.hpp>
#include <rmctrl_msgs/msg/arm_state_data.hpp>
#include <std_msgs/msg/int8.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using PoseCmd = rmctrl_msgs::msg::PoseCommand;
using JointState = rmctrl_msgs::msg::ArmStateData;
using namespace std::placeholders;

// 步骤类型
enum StepType { JOINT, GRIPPER_OPEN, GRIPPER_CLOSE };

// 步骤定义
struct Step {
    StepType type;
    std::vector<double> joint_angles; 
};

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
        : node_(node), sequence_active_(false), last_mode_(0), current_step_(0)
    {
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(0.1);
        arm_->setMaxAccelerationScalingFactor(0.1);
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

        // 创建发布者
        process_pub_ = node_->create_publisher<std_msgs::msg::Int8>("arm_ctrl_process", 10);

        // 动作1: 放在左边 (mode=1)
        left_place_sequence_ = {
            {GRIPPER_OPEN, {}},   // 1. 打开夹爪
            {JOINT, {1.484, 1.361, -0.523, 1.484, 1.570, 0.610}},  // 2. 预放置点
            {JOINT, {1.029, 1.361, -0.559, 1.170, 1.257, 0.576}},  // 3. 放置点
            {GRIPPER_CLOSE, {}},  // 4. 关闭夹爪（抓住物体）
            {JOINT, {1.030, 1.134, -0.646, 1.274, 1.134, 0.872}},  // 5. 抬升
            {JOINT, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}                // 6. 返回原点
        };

        // 动作2: 从左边拿起 (mode=2)
        left_pick_sequence_ = {
            {GRIPPER_OPEN, {}},   // 1. 打开夹爪
            {JOINT, {1.274, 1.309, -0.576, 1.344, 1.344, 0.838}},  // 2. 预抓取点
            {JOINT, {1.047, 1.344, -0.646, 1.187, 1.169, 0.803}},  // 3. 抓取点
            {GRIPPER_CLOSE, {}},  // 4. 关闭夹爪（抓起物体）
            {JOINT, {1.047, 1.135, -0.873, 1.414, 1.047, 1.292}},  // 5. 抬升
            {JOINT, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}                // 6. 返回原点
        };

        // 动作3: 放在右边 (mode=3)
        right_place_sequence_ = {
            {GRIPPER_OPEN, {}},
            {JOINT, {1.484, 1.361, -0.523, 1.484, 1.570, 0.610}},
            {JOINT, {1.029, 1.361, -0.559, 1.170, 1.257, 0.576}},
            {GRIPPER_CLOSE, {}},
            {JOINT, {1.030, 1.134, -0.646, 1.274, 1.134, 0.872}},
            {JOINT, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}
        };

        // 动作4: 从右边拿起 (mode=4)
        right_pick_sequence_ = {
            {GRIPPER_OPEN, {}},
            {JOINT, {1.484, 1.361, -0.523, 1.484, 1.570, 0.610}},
            {JOINT, {1.029, 1.361, -0.559, 1.170, 1.257, 0.576}},
            {GRIPPER_CLOSE, {}},
            {JOINT, {1.030, 1.134, -0.646, 1.274, 1.134, 0.872}},
            {JOINT, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}
        };
        // ====================================================

        open_gripper_sub_ = node_->create_subscription<Bool>(
            "open_gripper", 10, std::bind(&Commander::OpenGripperCallback, this, _1));

        joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "joint_commander", 10, std::bind(&Commander::JointCmdCallback, this, _1));

        pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
            "pose_commander", 10, std::bind(&Commander::PoseCmdCallback, this, _1));

        joint_state_sub_ = node_->create_subscription<JointState>(
            "joint_state_sub_from_mcu", 10,
            std::bind(&Commander::JointStateCallback, this, _1));
    }

    void GoToNamedTarget(const std::string &name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        PlanAndExecute(arm_);
    }

    bool GoToJointTarget(const std::vector<double> &joints)
    {
        if (joints.size() != 6) {
            RCLCPP_ERROR(node_->get_logger(), "关节角度数量错误: %zu, 应为6", joints.size());
            return false;
        }
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        return PlanAndExecute(arm_);
    }

    void GoToPoseTarget(double x, double y, double z, double roll, double pitch,
                        double yaw, bool cartesian_path = false)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();
        if (cartesian_path)
        {
            moveit_msgs::msg::RobotTrajectory trajectory;
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);

            double fraction = arm_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
            RCLCPP_INFO(node_->get_logger(), "笛卡尔规划执行比例 fraction = %.2f", fraction);
            if (fraction >= 0.95)
            {
                arm_->execute(trajectory);
                RCLCPP_INFO(node_->get_logger(), "执行笛卡尔规划");
            }
        }
        else
        {
            arm_->setPoseTarget(target_pose);
            PlanAndExecute(arm_);
        }
    }

    bool OpenGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open");
        return PlanAndExecute(gripper_);
    }

    bool CloseGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_close");
        return PlanAndExecute(gripper_);
    }

private:
    bool PlanAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        if (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            interface->execute(plan);
            return true;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "运动规划失败！");
            return false;
        }
    }

    void OpenGripperCallback(const Bool &msg)
    {
        if (msg.data)
            OpenGripper();
        else
            CloseGripper();
    }

    void JointCmdCallback(const FloatArray &msg)
    {
        auto joints = msg.data;
        if (joints.size() == 6)
            GoToJointTarget(joints);
    }

    void PoseCmdCallback(const PoseCmd &msg)
    {
        GoToPoseTarget(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.cartesian_path);
    }

    void PublishProcessStatus(int status)
    {
        std_msgs::msg::Int8 msg;
        msg.data = status;
        process_pub_->publish(msg);
        RCLCPP_DEBUG(node_->get_logger(), "发布 arm_ctrl_process: %d", status);
    }

    // 执行当前步骤
    bool ExecuteCurrentStep()
    {
        if (current_step_ < 0 || current_step_ >= static_cast<int>(current_sequence_.size())) {
            RCLCPP_ERROR(node_->get_logger(), "无效的步骤索引: %d", current_step_);
            return false;
        }
        const Step& step = current_sequence_[current_step_];
        PublishProcessStatus(current_step_ + 1);  // 步骤编号从1开始

        switch (step.type) {
            case JOINT:
                RCLCPP_INFO(node_->get_logger(), "执行关节运动步骤 %d/%zu", current_step_+1, current_sequence_.size());
                return GoToJointTarget(step.joint_angles);
            case GRIPPER_OPEN:
                RCLCPP_INFO(node_->get_logger(), "执行打开夹爪步骤 %d/%zu", current_step_+1, current_sequence_.size());
                return OpenGripper();
            case GRIPPER_CLOSE:
                RCLCPP_INFO(node_->get_logger(), "执行关闭夹爪步骤 %d/%zu", current_step_+1, current_sequence_.size());
                return CloseGripper();
            default:
                RCLCPP_ERROR(node_->get_logger(), "未知步骤类型");
                return false;
        }
    }

    // 推进状态机
    void AdvanceStateMachine()
    {
        bool success = ExecuteCurrentStep();

        if (!success)
        {
            RCLCPP_ERROR(node_->get_logger(), "步骤 %d 执行失败，序列终止", current_step_);
            // 尝试复位
            bool reset_success = GoToJointTarget({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            if (reset_success)
            {
                RCLCPP_INFO(node_->get_logger(), "复位成功，序列终止");
                PublishProcessStatus(-1);  // 复位成功
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "复位规划失败，请检查机械臂状态");
                PublishProcessStatus(-2);  // 复位失败
            }
            sequence_active_ = false;
            current_step_ = 0;
            current_sequence_.clear();
            return;
        }

        current_step_++;
        if (current_step_ >= static_cast<int>(current_sequence_.size()))
        {
            RCLCPP_INFO(node_->get_logger(), "-------------------------------状态机序列执行完成-------------------------------");
            PublishProcessStatus(0);
            sequence_active_ = false;
            current_step_ = 0;
            current_sequence_.clear();
        }
        else
        {
            AdvanceStateMachine();
        }
    }

    // MCU控制模式回调
    void JointStateCallback(const JointState::SharedPtr msg)
    {
        int mode = msg->arm_ctrl_mode;

        if (mode == 0)
        {
            if (sequence_active_)
                RCLCPP_WARN(node_->get_logger(), "模式变回0,但序列正在运行,忽略中断请求");
            return;
        }

        if (!sequence_active_ && mode != last_mode_)
        {
            std::vector<Step> selected_sequence;
            switch (mode)
            {
            case 1:
                selected_sequence = left_place_sequence_;
                RCLCPP_INFO(node_->get_logger(), "启动动作序列: 放在左边 (mode=1)");
                break;
            case 2:
                selected_sequence = left_pick_sequence_;
                RCLCPP_INFO(node_->get_logger(), "启动动作序列: 从左边拿起 (mode=2)");
                break;
            case 3:
                selected_sequence = right_place_sequence_;
                RCLCPP_INFO(node_->get_logger(), "启动动作序列: 放在右边 (mode=3)");
                break;
            case 4:
                selected_sequence = right_pick_sequence_;
                RCLCPP_INFO(node_->get_logger(), "启动动作序列: 从右边拿起 (mode=4)");
                break;
            default:
                RCLCPP_ERROR(node_->get_logger(), "未知的mode: %d", mode);
                return;
            }

            if (selected_sequence.empty())
            {
                RCLCPP_ERROR(node_->get_logger(), "mode %d 对应的序列为空，请先填入实际步骤", mode);
                return;
            }

            last_mode_ = mode;
            sequence_active_ = true;
            current_sequence_ = selected_sequence;
            current_step_ = 0;
            AdvanceStateMachine();
        }
        else if (sequence_active_)
        {
            RCLCPP_DEBUG(node_->get_logger(), "序列正在执行中,忽略新的mode=%d", mode);
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr process_pub_;

    // 状态机变量
    bool sequence_active_;
    int last_mode_;
    int current_step_;
    std::vector<Step> current_sequence_;

    // 预定义的四个动作序列（每个序列包含多个Step）
    std::vector<Step> left_place_sequence_;   // mode=1
    std::vector<Step> left_pick_sequence_;    // mode=2
    std::vector<Step> right_place_sequence_;  // mode=3
    std::vector<Step> right_pick_sequence_;   // mode=4
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    Commander commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
