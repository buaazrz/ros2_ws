#include "robot_controller_interface/controller_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include "robot_math/CartesianTrajectory.hpp"
#include "ros2_utility/data_comm.hpp"
#include "ros2_utility/data_logger.hpp"
#include "ros2_utility/file_utils.hpp"
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <iostream>
#include "ros2_utility/ros2_visual_tools.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_control_msgs/msg/robot_state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include <fstream>
#include <iomanip>

// using namespace robot_math;

namespace controllers
{
    class CartesianTrajImpPDPlusController : public controller_interface::ControllerInterface
    {
    public:
        using ACTION = robot_control_msgs::action::RobotMotion;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ACTION>;
        using BufferType = std::pair<std::shared_ptr<GoalHandle>, std::shared_ptr<robot_math::CartesianTrajectory>>;

        CartesianTrajImpPDPlusController() {}
        ~CartesianTrajImpPDPlusController() {}

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            dof_ = robot_->dof;
            
            node_->get_parameter_or<std::vector<double>>("Kx", Kx_vec_, {100.0, 100.0, 100.0, 50.0, 50.0, 50.0});
            node_->get_parameter_or<std::vector<double>>("Bx", Bx_vec_, {10.0, 10.0, 10.0, 5.0, 5.0, 5.0});
            node_->get_parameter_or<std::vector<double>>("Kn", Kn_vec_, {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
            node_->get_parameter_or<std::vector<double>>("Bn", Bn_vec_, {6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0});
            
            Kx_in_box_.set(Kx_vec_);
            Bx_in_box_.set(Bx_vec_);
            Kn_in_box_.set(Kn_vec_);
            Bn_in_box_.set(Bn_vec_);

            parameters_callback_handle_ = node_->add_on_set_parameters_callback(
                [&](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
                {
                    for (const auto &parameter : parameters)
                    {
                        if (parameter.get_name() == "Kx") Kx_in_box_.set([=](auto &value) { value = parameter.as_double_array(); });
                        else if (parameter.get_name() == "Bx") Bx_in_box_.set([=](auto &value) { value = parameter.as_double_array(); });
                        else if (parameter.get_name() == "Kn") Kn_in_box_.set([=](auto &value) { value = parameter.as_double_array(); });
                        else if (parameter.get_name() == "Bn") Bn_in_box_.set([=](auto &value) { value = parameter.as_double_array(); });
                    }
                    auto result = rcl_interfaces::msg::SetParametersResult();
                    result.successful = true;
                    return result;
                });
                
            return CallbackReturn::SUCCESS;
        }
        
        Eigen::VectorXd saturate_torque(const Eigen::VectorXd &tau_d_calculated, const Eigen::VectorXd &tau_J_d, double tol = 1.0)
        {
            Eigen::VectorXd tau_d_saturated(dof_);
            for (int i = 0; i < dof_; i++)
            {
                double difference = tau_d_calculated[i] - tau_J_d[i];
                tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, tol), -tol);
            }
            return tau_d_saturated;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            time_ = 0;
            traj_time_ = 0;
            real_time_buffer_.reset();
            tau_d.setZero();

            robot_state_publisher_ = node_->create_publisher<robot_control_msgs::msg::RobotState>("robot_states", rclcpp::SensorDataQoS());
            real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<robot_control_msgs::msg::RobotState>>(robot_state_publisher_);

            const std::vector<double> &q_vec = state_->get<double>("position");
            qd_ = Eigen::Map<const Eigen::VectorXd>(q_vec.data(), dof_).eval(); 
            dqd_ = Eigen::VectorXd::Zero(dof_);
            ddqd_ = Eigen::VectorXd::Zero(dof_);

            // const std::vector<double> &pose = state_->get<double>("pose");
            // Eigen::Matrix4d T = robot_math::pose_to_tform(pose);
            
            // Rd_ = T.block(0, 0, 3, 3);
            // pd_ = T.block(0, 3, 3, 1);

            Eigen::Matrix4d Tb_tmp; 
            forward_kinematics(robot_, q_vec, Tb_tmp);
            Rd_ = Tb_tmp.block(0, 0, 3, 3);
            pd_ = Tb_tmp.block(0, 3, 3, 1);
            
            wd_ = Eigen::Vector3d::Zero();
            vd_ = Eigen::Vector3d::Zero();
            ddxd_ = Eigen::Vector6d::Zero();

            Thb_ = Eigen::Matrix6d::Identity();
            dThb_ = Eigen::Matrix6d::Zero();

            M_.resize(dof_, dof_);     M_.setZero();
            C_.resize(dof_, dof_);     C_.setZero();
            dM_.resize(dof_, dof_);    dM_.setZero();
        

            tau_task_ = Eigen::VectorXd::Zero(dof_);
            tau_null_ = Eigen::VectorXd::Zero(dof_);
            tau_cmd_ = Eigen::VectorXd::Zero(dof_);

            auto handle_goal =[this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ACTION::Goal> goal) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };

            auto handle_cancel = [this](const std::shared_ptr<GoalHandle> goal_handle) {
                return rclcpp_action::CancelResponse::ACCEPT;
            };

            auto handle_accepted = [this](const std::shared_ptr<GoalHandle> goal_handle) {
                auto trajectory = std::make_shared<robot_math::CartesianTrajectory>();
                trajectory->set_traj(goal_handle->get_goal()->target_position.data);
                traj_time_ = 0.0; 
                real_time_buffer_.writeFromNonRT({goal_handle, trajectory});

                double dt = 0.001;

                std::ofstream outfile("/tmp/reference_trajectory.csv"); // Windows下可以改为 "C:\\temp\\reference_trajectory.csv" 之类的路径
                if (outfile.is_open())
                {
                    // 写入 CSV 表头（时间, x, y, z, qw, qx, qy, qz）
                    outfile << "time,x,y,z,qw,qx,qy,qz\n";
                    
                    for (double t = 0; t <= trajectory->total_time(); t += dt)
                    {
                        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
                        Eigen::Vector6d V, dV;
                        
                        // 评估 t 时刻的理想位姿
                        trajectory->evaluate(t, T, V, dV);

                        // 提取平移矩阵 (位置 X, Y, Z)
                        double x = T(0, 3);
                        double y = T(1, 3);
                        double z = T(2, 3);

                        // 提取旋转矩阵并转为四元数 (姿态 W, X, Y, Z)
                        Eigen::Quaterniond q(T.block<3, 3>(0, 0));

                        // 将数据以高精度写入 CSV 文件
                        outfile << std::fixed << std::setprecision(6)
                                << t << ","
                                << x << "," << y << "," << z << ","
                                << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << "\n";
                    }
                    outfile.close();
                    RCLCPP_INFO(node_->get_logger(), "Reference trajectory saved to /tmp/reference_trajectory.csv");
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to open file for trajectory saving!");
                }
            };

            call_back_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            this->action_server_ = rclcpp_action::create_server<ACTION>(
                node_, "~/goal", handle_goal, handle_cancel, handle_accepted, 
                rcl_action_server_get_default_options(), call_back_group_);

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            action_server_ = nullptr;
            real_time_publisher_ = nullptr;
            robot_state_publisher_ = nullptr;
            return CallbackReturn::SUCCESS;
        }

        void update(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            time_ += period.seconds();

            auto start_time = std::chrono::high_resolution_clock::now();

            std::vector<double> &tau_cmd_vec = command_->get<double>("torque");
            // auto &tau_d_vector = state_->get<double>("torque");
            command_->get<int>("mode")[0] = 3; 

            const std::vector<double> &q_vec = state_->get<double>("position");
            const std::vector<double> &dq_vec = state_->get<double>("velocity");
            // const std::vector<double> &pose = state_->get<double>("pose");
            // Eigen::Matrix4d T = pose_to_tform(pose);

            
            Eigen::Map<const Eigen::VectorXd> q(q_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> dq(dq_vec.data(), dof_);
            Eigen::Map<Eigen::VectorXd> tau_cmd(tau_cmd_vec.data(), dof_);
            std::fill(tau_cmd_vec.begin(), tau_cmd_vec.end(), 0);

            
            if (q.hasNaN() || dq.hasNaN()) {
                RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                    "检测到关节状态为 NaN！");
                Eigen::Map<Eigen::VectorXd> tau_cmd(tau_cmd_vec.data(), dof_);
                tau_cmd.setZero(); 
                return;
            }

            Kx_in_box_.try_get([=](auto const &value) { Kx_vec_ = value; });
            Bx_in_box_.try_get([=](auto const &value) { Bx_vec_ = value; });
            Kn_in_box_.try_get([=](auto const &value) { Kn_vec_ = value; });
            Bn_in_box_.try_get([=](auto const &value) { Bn_vec_ = value; });
            Kx_ = Eigen::Map<Eigen::VectorXd>(Kx_vec_.data(), 6);
            Bx_ = Eigen::Map<Eigen::VectorXd>(Bx_vec_.data(), 6);
            Kn_ = Eigen::Map<Eigen::VectorXd>(Kn_vec_.data(), dof_);
            Bn_ = Eigen::Map<Eigen::VectorXd>(Bn_vec_.data(), dof_);

            m_c_g_matrix(robot_, q_vec, dq_vec, M_, C_, g_, Jb_, dJb_, dM_, dTb_, Tb_);
            R_ = Tb_.block(0, 0, 3, 3); 
            p_ = Tb_.block(0, 3, 3, 1); 

            auto handle_pair = *real_time_buffer_.readFromRT();
            auto goal_handle = handle_pair.first;
            auto trajectory = handle_pair.second;

            dVd.setZero();

            if (goal_handle && goal_handle->is_active())
            {
                if (goal_handle->is_canceling())
                {
                    auto result = std::make_shared<ACTION::Result>();
                    result->success = false;
                    goal_handle->canceled(result);
                    vd_.setZero(); wd_.setZero(); ddxd_.setZero();
                }
                else
                {
                    traj_time_ += period.seconds();
                    Eigen::Matrix4d Td_curr;
                    Eigen::Vector6d Vd_curr, dVd_curr;

                    trajectory->evaluate(traj_time_, Td_curr, Vd_curr, dVd_curr);

                    Rd_ = Td_curr.block(0, 0, 3, 3);
                    pd_ = Td_curr.block(0, 3, 3, 1);
                    wd_ = Vd_curr.head(3);  // 角速度
                    vd_ = Vd_curr.tail(3);  // 线速度
                    dVd = dVd_curr;


                    Eigen::Vector3d pos_err = pd_ - p_;
                    Eigen::Vector3d rot_err = robot_math::logR(R_.transpose() * Rd_);
                    if (pos_err.norm() < 1e-3 && rot_err.norm() < 1e-2 && traj_time_ >= trajectory->total_time())
                    {
                        auto result = std::make_shared<ACTION::Result>();
                        result->success = true;
                        goal_handle->succeed(result);
                        vd_.setZero(); wd_.setZero(); ddxd_.setZero();
                    }
                }
            }
            else
            {
                vd_.setZero(); wd_.setZero(); ddxd_.setZero();
            }
            
            Eigen::Vector3d w = (Jb_ * dq).head(3);
            Eigen::Vector3d v = (Jb_ * dq).tail(3);

            xe_.head(3) = robot_math::logR(R_.transpose() * Rd_);
            xe_.tail(3) = R_.transpose() * (pd_ - p_);
            dxe_.head(3) = R_.transpose() * wd_ - w;
            dxe_.tail(3) = R_.transpose() * vd_ - v;

            double max_trans_err = 0.03; 
            if (xe_.tail(3).norm() > max_trans_err) {
                xe_.tail(3) = xe_.tail(3).normalized() * max_trans_err;
            }
            double max_rot_err = 0.1;
            if (xe_.head(3).norm() > max_rot_err) {
                xe_.head(3) = xe_.head(3).normalized() * max_rot_err;
            }

            ddxd_.head(3) = R_.transpose() * (dVd.head(3) - (R_ * w).cross(wd_));
            ddxd_.tail(3) = R_.transpose() * (dVd.tail(3) - (R_ * v).cross(vd_));

            ddxc_ = ddxd_ + robot_math::A_x_inv(Jb_,M_)*(robot_math::Mu_x_X(Jb_, M_, dJb_, C_, dxe_)) + Bx_.asDiagonal() * dxe_ + Kx_.asDiagonal() * xe_;
            
            tau_task_ = M_ * robot_math::J_sharp_X(Jb_, M_,  (ddxc_- dJb_* dq));

            Eigen::LDLT<Eigen::MatrixXd> ldlt(M_);
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dof_, dof_);
            tau_null_ = M_ * ((I - (robot_math::J_sharp(Jb_, M_) * Jb_)) * ldlt.solve(Bn_.asDiagonal() * (-dq))); 

            tau_cmd = tau_task_ + tau_null_;
            tau_cmd = saturate_torque(tau_cmd, tau_d);
            tau_d = tau_cmd;

            RCLCPP_INFO(get_node()->get_logger(), "tau_task_: %.6f %.6f %.6f %.6f %.6f %.6f %.6f Nm", 
            tau_task_[0], tau_task_[1], tau_task_[2], tau_task_[3],tau_task_[4], tau_task_[5],tau_task_[6]);

            RCLCPP_INFO(get_node()->get_logger(), "tau_null_: %.6f %.6f %.6f %.6f %.6f %.6f %.6f Nm", 
            tau_null_[0], tau_null_[1], tau_null_[2], tau_null_[3],tau_null_[4], tau_null_[5],tau_null_[6]);

            publish_robot_state(t, q_vec, dq_vec);

            cal_time_ = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
            // RCLCPP_INFO(get_node()->get_logger(), "cal_time: %.6f seconds", cal_time_);

        }
    
    private:
        void publish_robot_state(const rclcpp::Time &t, 
                                 const std::vector<double> &q, 
                                 const std::vector<double> &dq)
        {
            if (!real_time_publisher_) return;

            robot_control_msgs::msg::RobotState msg;
            msg.header.stamp = t;
            
            std::fill_n(std::back_inserter(msg.robot_state), 28, 0);
            std::copy(q.begin(), q.end(), msg.robot_state.begin());
            std::copy(dq.begin(), dq.end(), msg.robot_state.begin() + 7);

            if (real_time_publisher_->trylock())
            {
                real_time_publisher_->msg_ = msg;
                real_time_publisher_->unlockAndPublish();
            }
        }
    protected:
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
        rclcpp::Publisher<robot_control_msgs::msg::RobotState>::SharedPtr robot_state_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<robot_control_msgs::msg::RobotState>> real_time_publisher_;
        rclcpp::CallbackGroup::SharedPtr call_back_group_;
        rclcpp_action::Server<ACTION>::SharedPtr action_server_;
        realtime_tools::RealtimeBuffer<BufferType> real_time_buffer_;

        int dof_;
        double time_, traj_time_;
        Eigen::MatrixXd M_, C_, Jb_, dJb_, dM_, Jh_, dJh_;
        Eigen::VectorXd g_;
        Eigen::Matrix4d Tb_, dTb_;
        Eigen::VectorXd Kx_, Bx_, Kn_, Bn_;
        Eigen::VectorXd tau_cmd_, tau_task_, tau_null_;
        Eigen::VectorXd qd_, dqd_, ddqd_, qe_, dqe_;
        Eigen::Vector6d xe_, dxe_, ddxd_, ddxc_, dVd;
        Eigen::Matrix3d Rd_, R_;
        Eigen::Matrix6d Thb_, dThb_;
        Eigen::Vector3d pd_, p_, wd_, vd_;
        Eigen::Vector7d tau_d;
        double cal_time_;

        // 线程安全的参数盒子
        realtime_tools::RealtimeBox<std::vector<double>> Kx_in_box_, Bx_in_box_, Kn_in_box_, Bn_in_box_;
        std::vector<double> Kx_vec_, Bx_vec_, Kn_vec_, Bn_vec_;
    };
} // namespace controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controllers::CartesianTrajImpPDPlusController, controller_interface::ControllerInterface)