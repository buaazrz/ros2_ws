#include "robot_hardware_interface/robot_interface.hpp"
#include "robot_math/robot_math.hpp"
#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <pluginlib/class_list_macros.hpp>

// 遨博 Aubo SDK 头文件（按 8.6 章节）
#include <aubo/aubo_api.h>
#include <aubo/robot/robot_interface.h>
#include <aubo/robot/motion_control.h>
#include <aubo/robot/robot_state.h>
#include <aubo/robot/io_control.h>

using namespace robot_math;
namespace hardwares
{
// 继承 ros2_control 标准 RobotInterface 基类
class AuboRobot : public hardware_interface::RobotInterface
{
public:
  AuboRobot() : pre_dq_(6)
  {
  }

  ~AuboRobot() override
  {
  }

  /**
   * @brief 写控制指令：下发关节/位姿/速度指令到AUBO控制器
   */
  void write(const rclcpp::Time &t, const rclcpp::Duration &period) override
  {
    hardware_interface::RobotInterface::write(t, period);
    double dt = period.seconds();

    // 获取控制模式与指令
    int mode = command_.get<int>("mode")[0];
    auto &cmd_joint_pos = command_.get<double>("position");
    auto &cmd_tcp_pose  = command_.get<double>("pose");
    auto &cmd_joint_vel  = command_.get<double>("velocity");

    // 运动参数（可从参数服务器加载）
    const double vel  = 0.5;
    const double acc  = 0.5;

    // 模式 0: TCP位姿伺服L
    // 模式 1: 关节位置伺服J
    // 模式 2: 关节速度控制J
    switch (mode)
    {
      case 0:
        motion_control_->moveL(cmd_tcp_pose, vel, acc);
        break;
      case 1:
        motion_control_->moveJ(cmd_joint_pos, vel, acc);
        break;
      case 2:
        motion_control_->speedJ(cmd_joint_vel, acc, dt);
        break;
      default:
        RCLCPP_WARN(node_->get_logger(), "AUBO unknown mode: %d", mode);
        break;
    }
  }

  /**
   * @brief 急停/停止判断（用IO状态）
   */
  bool is_stop() override
  {
    return state_.get<bool>("io")[0];
  }

  /**
   * @brief 读状态：从AUBO控制器读取关节、速度、加速度、位姿、IO
   */
  void read(const rclcpp::Time &t, const rclcpp::Duration &period) override
  {
    hardware_interface::RobotInterface::read(t, period);

    auto &q       = state_.get<double>("position");
    auto &io_state = state_.get<bool>("io");
    auto &dq      = state_.get<double>("velocity");
    auto &ddq     = state_.get<double>("acceleration");
    auto &pose    = state_.get<double>("pose");

    double dt = period.seconds();

    // ========== 从AUBO SDK读取状态（8.6 RobotState / IoControl）==========
    q    = robot_state_->getJointState();
    dq   = robot_state_->getJointSpeed();
    pose = robot_state_->getTcpPose();

    // 读取标准数字输出 DO0 / DO1
    io_state[0] = io_control_->getStandardDigitalOutput(0);
    io_state[1] = io_control_->getStandardDigitalOutput(1);

    // 计算关节加速度
    if (dt > 0)
    {
      for (int i = 0; i < 6; i++)
      {
        ddq[i] = (dq[i] - pre_dq_[i]) / dt;
        pre_dq_[i] = dq[i];
      }
    }
    else
    {
      pre_dq_ = dq;
      std::fill(ddq.begin(), ddq.end(), 0);
    }
  }

  /**
   * @brief 配置阶段：连接AUBO控制器，获取SDK接口
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override
  {
    if (RobotInterface::on_configure(previous_state) != CallbackReturn::SUCCESS)
      return CallbackReturn::FAILURE;

    // 从参数获取机器人IP
    node_->get_parameter_or<std::string>("robot_ip", robot_ip_, "");
    if (robot_ip_.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "AUBO robot_ip not set");
      return CallbackReturn::FAILURE;
    }

    try
    {
      // 1. 创建Aubo主API
      aubo_api_ = std::make_shared<arcs::common_interface::AuboApi>();

      // 2. 获取机器人接口
      auto robot_names = aubo_api_->getRobotNames();
      robot_interface_ = aubo_api_->getRobotInterface(robot_names.front());

      // 3. 获取核心模块（8.6章节）
      motion_control_ = robot_interface_->getMotionControl();
      robot_state_    = robot_interface_->getRobotState();
      io_control_     = robot_interface_->getIoControl();

      RCLCPP_INFO(node_->get_logger(), "AUBO connected: %s", robot_ip_.c_str());
    }
    catch (const arcs::common_interface::AuboException &e)
    {
      RCLCPP_ERROR(node_->get_logger(), "AUBO connect failed: %s", e.what());
      return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief 关机：停止运动
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override
  {
    RobotInterface::on_shutdown(previous_state);
    if (motion_control_)
      motion_control_->stopMove();

    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief 激活：初始化速度缓存
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override
  {
    if (RobotInterface::on_activate(previous_state) == CallbackReturn::SUCCESS)
    {
      std::fill(pre_dq_.begin(), pre_dq_.end(), 0);
      return CallbackReturn::SUCCESS;
    }
    return CallbackReturn::FAILURE;
  }

  /**
   * @brief 失活：停止运动
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override
  {
    RobotInterface::on_deactivate(previous_state);
    if (motion_control_)
      motion_control_->stopMove();

    return CallbackReturn::SUCCESS;
  }

protected:
  std::string robot_ip_;
  std::vector<double> pre_dq_;    // 上一周期速度（算加速度）

  // Aubo SDK 核心接口（来自8.6章节）
  std::shared_ptr<arcs::common_interface::AuboApi>          aubo_api_;
  std::shared_ptr<arcs::common_interface::RobotInterface>    robot_interface_;
  std::shared_ptr<arcs::common_interface::MotionControl>    motion_control_;
  std::shared_ptr<arcs::common_interface::RobotState>       robot_state_;
  std::shared_ptr<arcs::common_interface::IoControl>        io_control_;
};

} // namespace hardwares

// 导出插件（与UR完全一致）
PLUGINLIB_EXPORT_CLASS(hardwares::AuboRobot, hardware_interface::RobotInterface)