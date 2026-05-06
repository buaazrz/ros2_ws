#include "robot_controller_interface/controller_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include "robot_math/CartesianTrajectory.hpp"
#include "ros2_utility/data_logger.hpp"
#include "ros2_utility/file_utils.hpp"
#include "math.h"
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
#include "optimization.h" 

// 定义传给 ALGLIB 目标函数回调的动态参数结构体
struct OptParams {
    double Q_weight;
    double R_weight;
    double xe;
    double dx_B;       // 相当于阻尼力 D_c * dx_r
    double F_target;   // 期望力 F_des + F_com
    double K_min;
};

// ALGLIB 目标函数与梯度计算回调
static void alglib_grad_callback(const alglib::real_1d_array &x, double &func, alglib::real_1d_array &grad, void *ptr) 
{
    // 解析传入的参数
    OptParams* params = static_cast<OptParams*>(ptr);
    double K = x[0];
    double Q = params->Q_weight;
    double R = params->R_weight;
    double xe = params->xe;
    double dx_B = params->dx_B;
    double F_target = params->F_target;
    double K_min = params->K_min;

    // 根据公式(4)计算当前预估的外力
    double F_ext_est = K * xe - dx_B;
    // 力跟踪误差
    double force_err = F_ext_est - F_target;

    // 代价函数 J = 0.5 * Q * (F_ext - F_target)^2 + 0.5 * R * (K - K_min)^2
    func = 0.5 * Q * force_err * force_err + 0.5 * R * (K - K_min) * (K - K_min);
    
    // 对 K 求偏导计算梯度
    grad[0] = Q * xe * force_err + R * (K - K_min);
}

// using namespace robot_math;

namespace controllers
{
    class VariableImpedanceController : public controller_interface::ControllerInterface
    {
    public:
        using ACTION = robot_control_msgs::action::RobotMotion;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ACTION>;
        using BufferType = std::pair<std::shared_ptr<GoalHandle>, std::shared_ptr<robot_math::CartesianTrajectory>>;

        VariableImpedanceController() : f_filter_(6, 15) {}
        ~VariableImpedanceController() 
        {
            if (data_logger_)
               data_logger_->save("/home/wjc/experiment_logs/variable_imp_controller/", "variable_imp_controller");
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            dof_ = robot_->dof;
            
            node_->get_parameter_or<std::vector<double>>("Kx", Kx_vec_, {500.0, 500.0, 500.0, 2500.0, 2500.0, 2500.0});
            node_->get_parameter_or<std::vector<double>>("Bx", Bx_vec_, {30.0, 30.0, 30.0, 150.0, 150.0, 150.0});
            node_->get_parameter_or<std::vector<double>>("Kn", Kn_vec_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            node_->get_parameter_or<std::vector<double>>("Bn", Bn_vec_, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
            node_->get_parameter_or<double>("Q_weight", Q_weight_, 1.0);
            node_->get_parameter_or<double>("R_weight", R_weight_, 0.001);
            node_->get_parameter_or<double>("F_des_z", F_des_z_, 2.0);
            node_->get_parameter_or<double>("F_max_z", F_max_z_, 10.0);
            node_->get_parameter_or<double>("Kp_f", Kp_f_, 0.0);
            node_->get_parameter_or<double>("Ki_f", Ki_f_, 0.0);
            node_->get_parameter_or<double>("Kz_min", Kz_min_, 100.0);
            node_->get_parameter_or<double>("Kz_max", Kz_max_, 2500.0);

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
            force_int_z_ = 0.0;
            real_time_buffer_.reset();
            tau_d.setZero();
            f_filter_.reset();

            robot_state_publisher_ = node_->create_publisher<robot_control_msgs::msg::RobotState>("robot_states", rclcpp::SensorDataQoS());
            real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<robot_control_msgs::msg::RobotState>>(robot_state_publisher_);

            const std::vector<double> &q_vec = state_->get<double>("position");
            qd_ = Eigen::Map<const Eigen::VectorXd>(q_vec.data(), dof_).eval(); 
            dqd_ = Eigen::VectorXd::Zero(dof_);
            ddqd_ = Eigen::VectorXd::Zero(dof_);

            const std::vector<double> &pose = state_->get<double>("pose");
            Eigen::Matrix4d T = robot_math::pose_to_tform(pose);
            Eigen::Matrix4d Tb_tmp; 
            robot_math::forward_kinematics(robot_, q_vec, Tb_tmp);
            Rd_ = Tb_tmp.block(0, 0, 3, 3);
            pd_ = Tb_tmp.block(0, 3, 3, 1);
            
            wd_ = Eigen::Vector3d::Zero();
            vd_ = Eigen::Vector3d::Zero();
            ddxd_ = Eigen::Vector6d::Zero();
            tau_task_ = Eigen::VectorXd::Zero(dof_);
            tau_null_ = Eigen::VectorXd::Zero(dof_);
            tau_cmd_ = Eigen::VectorXd::Zero(dof_);

            sensor_weight_ = 0.61569; 
            sensor_cog_vec_ = {-0.0004 ,  -0.0000 ,   0.0026};
            sensor_offset_vec_ = { -5.8497   , 5.6095,   -9.4235  ,  0.4396 ,   0.2853   , 0.2743};
            T_sensor_ = Eigen::Matrix4d::Identity();
            T_sensor_ << 1,  0,  0, 0,
             0, -1,  0, 0,
             0,  0, -1, 0,
             0,  0,  0, 1;

            x_opt_.setlength(1);
            bndl_.setlength(1);
            bndu_.setlength(1);
            c_.setlength(2, 2);
            ct_.setlength(2);
            scale_.setlength(1);
            
            bndl_[0] = Kz_min_;
            bndu_[0] = Kz_max_;
            ct_[0] = 1; ct_[1] = -1;
            scale_[0] = 1.0;
            
            // 执行一次初始建档 (黑盒生成)
            x_opt_[0] = Kx_vec_[5];
            alglib::minbleiccreate(x_opt_, alglib_state_);
            alglib::minbleicsetbc(alglib_state_, bndl_, bndu_);
            alglib::minbleicsetscale(alglib_state_, scale_);
            alglib::minbleicsetcond(alglib_state_, 0.0, 0.0, 1e-4, 10); // maxits=10

            auto handle_goal =[this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ACTION::Goal> goal) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };

            auto handle_cancel = [this](const std::shared_ptr<GoalHandle> goal_handle) {
                return rclcpp_action::CancelResponse::ACCEPT;
            };

            auto handle_accepted = [this](const std::shared_ptr<GoalHandle> goal_handle) {
                auto trajectory = std::make_shared<robot_math::CartesianTrajectory>();
                const std::vector<double> &pose_current = state_->get<double>("pose");
                std::vector<double> full_traj_data;
                full_traj_data.push_back(0.0); 
                for(int i = 0; i < 6; ++i) {
                    full_traj_data.push_back(pose_current[i]); 
                }
                const auto& goal_data = goal_handle->get_goal()->target_position.data;
                full_traj_data.insert(full_traj_data.end(), goal_data.begin(), goal_data.end());
                trajectory->set_traj(full_traj_data);
                
                traj_time_ = 0.0; 
                real_time_buffer_.writeFromNonRT({goal_handle, trajectory});
            };

            call_back_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            this->action_server_ = rclcpp_action::create_server<ACTION>(
                node_, "~/goal", handle_goal, handle_cancel, handle_accepted, 
                rcl_action_server_get_default_options(), call_back_group_);
            
            pose_ = Eigen::Vector6d::Zero();
            force_ = Eigen::Vector6d::Zero();
            dq_ = Eigen::Vector7d::Zero();
            q_ = Eigen::Vector7d::Zero();
            data_logger_ = std::make_unique<DataLogger>(
                std::initializer_list<DataInfo>{
                    DATA_WRAPPER(time_),
                    DATA_WRAPPER(cal_time_),
                    // DATA_WRAPPER(pose_),
                    // DATA_WRAPPER(q_),
                    // DATA_WRAPPER(tau_d),
                    // DATA_WRAPPER(dq_),
                    // DATA_WRAPPER(force_),
                    DATA_WRAPPER(Kx_(5)),
                    // DATA_WRAPPER(tau_null_),
                    // DATA_WRAPPER(xe_),
                    // DATA_WRAPPER(dxe_),
                    // DATA_WRAPPER(ddxd_),
                },
                std::initializer_list<ExperimentContext>{
                    CONFIG_WRAPPER(Kx_vec_),
                    CONFIG_WRAPPER(Bx_vec_),
                    CONFIG_WRAPPER(Kn_vec_),
                    CONFIG_WRAPPER(Bn_vec_),
                },
                1000);

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
            const std::vector<double> &q_vec = state_->get<double>("position");
            const std::vector<double> &dq_vec = state_->get<double>("velocity");
            const std::vector<double> &pose_vec = state_->get<double>("pose");
            auto &force_vec = com_state_->at("ft_sensor")->get<double>("force");
            command_->get<int>("mode")[0] = 3; 
            pose_ = Eigen::Map<const Eigen::Vector6d>(pose_vec.data());
            force_ = Eigen::Map<const Eigen::Vector6d>(force_vec.data());
            dq_ = Eigen::Map<const Eigen::Vector7d>(dq_vec.data());
            q_ = Eigen::Map<const Eigen::Vector7d>(q_vec.data());
            Eigen::Matrix4d T = robot_math::pose_to_tform(pose_vec);
            // R_ = T.block(0, 0, 3, 3); 
            // p_ = T.block(0, 3, 3, 1); 
         
            Eigen::Map<const Eigen::VectorXd> q(q_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> dq(dq_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> force(force_vec.data(), 6);
            Eigen::Map<Eigen::VectorXd> tau_cmd(tau_cmd_vec.data(), dof_);
            std::fill(tau_cmd_vec.begin(), tau_cmd_vec.end(), 0);

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
            
            Eigen::Vector6d raw_compensated = robot_math::get_ext_force(
                force_vec, 
                sensor_weight_, 
                sensor_offset_vec_, 
                sensor_cog_vec_, 
                T_sensor_, 
                Tb_
            );
            force_.head(3) = raw_compensated.tail(3); 
            force_.tail(3) = raw_compensated.head(3); 
            Eigen::Matrix3d R_tcp_to_sensor = T_sensor_.block<3,3>(0,0);
            force_.head(3) = R_tcp_to_sensor * force_.head(3);
            force_.tail(3) = R_tcp_to_sensor * force_.tail(3);    
 
            f_filter_.filtering(force_.data(), force_.data());

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
                    wd_ = Vd_curr.head(3); 
                    vd_ = Vd_curr.tail(3); 
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

            ddxd_.head(3) = R_.transpose() * (dVd.head(3) - (R_ * w).cross(wd_));
            ddxd_.tail(3) = R_.transpose() * (dVd.tail(3) - (R_ * w).cross(vd_));

            double F_ext_z = force_(2); 
            double F_err_z = F_des_z_ - F_ext_z;
            // std::cerr << "F_des_z: " << F_des_z_ << std::endl;

            force_int_z_ += F_err_z * period.seconds();
            double F_com_z = Kp_f_ * F_err_z + Ki_f_ * force_int_z_;
            double xe_z = xe_(5);       
            double dxe_z = dxe_(5);
            double B_z = Bx_(5);

            if (std::abs(xe_z) > 1e-3) 
            {
                params_.Q_weight = Q_weight_;
                params_.R_weight = R_weight_;
                params_.xe = xe_z;
                params_.dx_B = B_z * dxe_z;
                params_.F_target = F_des_z_ + F_com_z;
                params_.K_min = Kz_min_;

                try 
                {
                    x_opt_[0] = Kx_(5);

                    // 2. 更新约束系数
                    c_[0][0] = xe_z; c_[0][1] = -F_max_z_ + params_.dx_B; 
                    c_[1][0] = xe_z; c_[1][1] =  F_max_z_ + params_.dx_B; 


                    // 3. 将新约束和新起点注入原有黑盒 (绝对不能再调 create!)
                    alglib::minbleicsetlc(alglib_state_, c_, ct_);
                    alglib::minbleicrestartfrom(alglib_state_, x_opt_); // <--- 高级接口：复用内存并更换起点

                    // 4. 执行优化
                    alglib::minbleicoptimize(alglib_state_, alglib_grad_callback, NULL, &params_);
                    
                    // 5. 提取结果 (使用 buffered 版本，防止内部 malloc)
                    alglib::minbleicresultsbuf(alglib_state_, x_opt_, rep_);

                    // 提取结果并注入底层阻抗控制器
                    if (int(rep_.terminationtype) > 0) 
                    {
                        Kx_vec_[5] = x_opt_[0];
                        Kx_(5) = x_opt_[0];
                        Bx_(5) = 1.42 * sqrt(Kx_(5));
                    } 
                    else 
                    {
                        // 如果无解(通常是因为力太大,约束打架),强制设为最小刚度
                        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, 
                            "ALGLIB failed (Code: %d). Fallback to K_min.", int(rep_.terminationtype));
                        Kx_vec_[5] = Kz_min_;
                        Kx_(5) = Kz_min_;
                    }
                }
                catch(alglib::ap_error alglib_exception)
                {
                    RCLCPP_WARN(node_->get_logger(), "ALGLIB exception: %s", alglib_exception.msg.c_str());
                }
            }

            ddxc_ = ddxd_ + robot_math::A_x_inv(Jb_, M_) * (robot_math::Mu_x_X(Jb_, M_, dJb_, C_, dxe_) + Bx_.asDiagonal() * dxe_ + Kx_.asDiagonal() * xe_);
            tau_task_ = M_ * robot_math::J_sharp(Jb_, M_) * (ddxc_- dJb_* dq);
            // tau_task_ = M_ * robot_math::J_sharp_X(Jb_, M_, ddxc_- dJb_* dq);
            Eigen::LDLT<Eigen::MatrixXd> ldlt(M_);
            tau_null_ = M_ * robot_math::null_proj(Jb_, M_, ldlt.solve(Bn_.asDiagonal() * (-dq)));

            tau_cmd = tau_task_ + tau_null_ ;
            tau_cmd = saturate_torque(tau_cmd, tau_d);
            tau_d = tau_cmd;

            publish_robot_state(t, q_vec, dq_vec, force_);

            cal_time_ = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
            data_logger_->record();

        }
    
    private:
        void publish_robot_state(const rclcpp::Time &t, 
            const std::vector<double> &q, 
            const std::vector<double> &dq,
            const Eigen::Vector6d &force // <--- 修改 1：参数类型改为 Eigen::Vector6d
        )
        {
        if (!real_time_publisher_) return;
        robot_control_msgs::msg::RobotState msg;
        msg.header.stamp = t;
        std::fill_n(std::back_inserter(msg.robot_state), 28, 0);
        std::copy(q.begin(), q.end(), msg.robot_state.begin());
        std::copy(dq.begin(), dq.end(), msg.robot_state.begin() + 7);
        std::copy(force.data(), force.data() + 6, msg.robot_state.begin() + 14);
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
        Eigen::MatrixXd M_, C_, Jb_, dJb_, dM_;
        Eigen::Vector6d pose_, force_;
        Eigen::VectorXd g_;
        Eigen::Matrix4d Tb_, dTb_;
        Eigen::VectorXd Kx_, Bx_, Kn_, Bn_;
        Eigen::VectorXd tau_cmd_, tau_task_, tau_null_;
        Eigen::VectorXd qd_, dqd_, ddqd_, qe_, dqe_, dq_, q_;
        Eigen::Vector6d xe_, dxe_, ddxd_, ddxc_, dVd;
        Eigen::Matrix3d Rd_, R_;
        Eigen::Vector3d pd_, p_, wd_, vd_;
        Eigen::Vector7d tau_d;
        std::vector<double> sensor_cog_vec_;
        std::vector<double> sensor_offset_vec_;
        double sensor_weight_;
        alglib::real_1d_array x_opt_;
        alglib::real_1d_array bndl_, bndu_;
        alglib::real_2d_array c_;
        alglib::integer_1d_array ct_;
        alglib::real_1d_array scale_;
        alglib::minbleicstate alglib_state_;
        alglib::minbleicreport rep_;
        OptParams params_;
        Eigen::Matrix4d T_sensor_;
        double F_des_z_, F_max_z_;
        double Kz_min_, Kz_max_;
        double Q_weight_, R_weight_;
        double Kp_f_, Ki_f_, force_int_z_;
        double cal_time_;
        std::unique_ptr<DataLogger> data_logger_;
        realtime_tools::RealtimeBox<std::vector<double>> Kx_in_box_, Bx_in_box_, Kn_in_box_, Bn_in_box_;
        std::vector<double> Kx_vec_, Bx_vec_, Kn_vec_, Bn_vec_;
        robot_math::MovingFilter<double> f_filter_;
    };
} // namespace controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controllers::VariableImpedanceController, controller_interface::ControllerInterface)