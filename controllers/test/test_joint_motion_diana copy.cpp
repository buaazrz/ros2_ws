#include <rclcpp/rclcpp.hpp>
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <vector>
#include <chrono>

using namespace std::chrono_literals;
using ACTION = robot_control_msgs::action::RobotMotion;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_joint_trajectory");
    
    // 注意名称对应上面的 JointMotionControllerDiana
    auto client = rclcpp_action::create_client<ACTION>(node, "JointMotionControllerDiana/goal");

    if (!client->wait_for_action_server(5s)) {
        RCLCPP_ERROR(node->get_logger(), "Action server not available");
        rclcpp::shutdown();
        return 1;
    }

    // =========================================================================
    // 这里填入你通过手动拖动示教记录下来的【真实关节角度】
    // 假设是 7 轴机械臂，请填入7个数值 (如果是6轴请删掉一列)
    // =========================================================================
    std::vector<std::vector<double>> target_joints = {
        {0.086574, -0.240955, -0.022051, 2.170200, -0.029145, -1.195115, -0.010282},
        {0.088420, -0.240979, -0.042472, 2.107930, 1.666095, -1.304459, -0.009923},
        {0.088444, -0.256175, 0.097144, 2.181057, 0.040363, -0.676677, -0.008365},
        {0.088396, -0.241051, 0.015388, 2.056541, 2.819457, -1.558237, -0.010114},
        {0.088396, -0.241051, 0.073703, 2.090313, -0.646117, -1.286818, -0.008604},
        {0.088396, -0.241003, 0.005393, 2.077969, 2.166269, -1.568088, -0.010162},
        {0.088396, -0.247354, 0.067352, 2.036048, -1.585585, -1.454813, -0.008293},
        {0.088444, -0.322735, 0.039213, 2.026125, 0.038973, -2.354708, -0.008389},
        {0.088396, -0.247378, 0.069676, 2.035856, -2.270052, -1.662140, -0.008317},
        {0.088396, -0.240955, -0.071306, 2.169792, 0.580061, -1.304483, -0.009923},
        {0.088444, -0.256079, 0.096713, 2.277027, 0.040794, 0.443824, -0.008389},
        {0.088444, -0.256270, 0.101267, 2.247977, 0.039812, 1.062258, -0.008389},
        {0.088444, -0.294524, 0.038062, 2.333209, 0.038997, -2.663039, -0.008413},
        {0.088612, -0.366238, 0.031495, 2.345793, 0.631329, -1.000515, -1.262226},
        {0.088204, -0.373404, 0.007910, 2.289539, -0.630754, -1.633402, -0.593722},
        {0.088204, -0.370312, -0.102872, 2.338242, -0.811020, -0.834270, 1.292283},
        {0.088564, -0.370504, 0.055679, 2.346823, 0.648179, -1.192862, 1.292211},
        {0.096353, -0.372973, 0.286016, 2.197260, 0.604604, -0.952937, -1.712809},
        {0.095826, -0.374243, -0.146064, 2.254880, -0.741392, -0.965713, 0.990425},
        {0.083506, -0.349268, -0.177630, 2.251189, -0.860060, -0.630730, 0.008964},
        {0.083530, -0.352192, 0.177990, 2.358400, 0.650456, -0.979471, -0.133121},
        {0.083554, -0.348861, 0.150930, 2.365519, 0.577664, -1.114677, -0.116534},
        {-0.413528, -0.306173, -0.068981, 2.341191, -0.547535, -0.952075, -0.101314},
        {0.097576, -0.298575, -0.119674, 2.259123, -2.227748, -0.880481, -0.102681},
        {0.097024, -0.305957, -0.026317, 2.299102, 0.252987, 0.781947, 1.581271},
        {0.097120, -0.306389, 0.486823, 2.247330, 1.141402, -0.850544, 0.872739},
        {0.097048, -0.380379, 0.220462, 2.437232, 0.504680, -1.836582, 0.324533},
        {0.096569, -0.396654, -0.216794, 2.437064, -0.918135, -1.861270, 0.324485},
        {0.097216, -0.264180, 0.380260, 2.378222, 0.176264, -0.357729, 0.323215},
        {0.096353, -0.261208, -0.229857, 2.319643, -0.950972, -0.687415, -0.804693},
        {0.097048, -0.261208, 0.728042, 2.357417, 1.026881, -0.686432, 0.012392},
    };

    std::vector<double> full_trajectory_data;
    double current_time = 0.0;
    
    double move_duration = 6.0; // 相邻点之间的移动耗时
    double stay_duration = 2.0; // 到达后原地停留记录数据的时长

    // 压入第一个点 (作为起点)
    full_trajectory_data.push_back(current_time);
    full_trajectory_data.insert(full_trajectory_data.end(), target_joints[0].begin(), target_joints[0].end());

    // 构建 Move 和 Stay 的时间序列
    for (size_t i = 0; i < target_joints.size(); ++i)
    {
        // 动作 A：从【上一状态】平滑移动到 target_joints[i]
        // 注：如果是 i=0，机械臂会从【未知的当前姿态】平滑移动到 target_joints[0]
        current_time += move_duration;
        full_trajectory_data.push_back(current_time);
        full_trajectory_data.insert(full_trajectory_data.end(), target_joints[i].begin(), target_joints[i].end());

        // 动作 B：在原姿态停留，触发服务端的 Data Logger
        current_time += stay_duration;
        full_trajectory_data.push_back(current_time);
        full_trajectory_data.insert(full_trajectory_data.end(), target_joints[i].begin(), target_joints[i].end());
    }
    auto goal_msg = ACTION::Goal();
    goal_msg.target_position.data = full_trajectory_data;

    RCLCPP_INFO(node->get_logger(), "Sending Joint Trajectory with %zu waypoints. Total execution time: %.1f seconds.", 
                target_joints.size(), current_time);

    auto handle_future = client->async_send_goal(goal_msg);
    auto result = rclcpp::spin_until_future_complete(node, handle_future);
    
    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
        rclcpp::shutdown(); return 0;
    }
    
    auto handle = handle_future.get();
    if(handle == nullptr) {
        RCLCPP_ERROR(node->get_logger(), "Goal rejected");
        rclcpp::shutdown(); return 0;
    }
    
    RCLCPP_INFO(node->get_logger(), "Goal accepted, waiting for execution to finish...");
    
    auto result_future = client->async_get_result(handle);
    rclcpp::spin_until_future_complete(node, result_future);
    
    if (result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node->get_logger(), "Trajectory execution succeeded!");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Trajectory execution failed!");
    }

    rclcpp::shutdown();
    return 0;
}