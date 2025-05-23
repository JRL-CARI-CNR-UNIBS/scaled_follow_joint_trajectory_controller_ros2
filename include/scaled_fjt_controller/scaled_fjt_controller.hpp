#ifndef SCALED_FJT_CONTROLLER
#define SCALED_FJT_CONTROLLER

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "scaled_fjt_controller/microinterpolator.h"
#include <std_msgs/msg/int16.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include <map>

namespace scaled_fjt_controller
{
class ScaledFjtController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  ScaledFjtController() = default;
  ~ScaledFjtController() override = default;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  
  rclcpp_action::GoalResponse   goal_received_callback (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJTrajAction::Goal> goal);
  rclcpp_action::CancelResponse goal_cancelled_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);
  void                          goal_accepted_callback (std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

  trajectory_msgs::msg::JointTrajectory trj_;
  std::shared_ptr<Microinterpolator> microinterpolator_;
  trajectory_msgs::msg::JointTrajectoryPoint current_point_;
  std::shared_ptr<sensor_msgs::msg::JointState> unscaled_js_msg_;

  double speed_ovr_;
  std::map<std::string,double> speed_ovr_map_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr> speed_ovr_sub_;

  void SpeedOvrCb(const std_msgs::msg::Int16 &msg, const std::string &topic);

protected:
  struct TimeData
  {
    TimeData() : time(rclcpp::Duration::from_seconds(0.0)),scaled_time(rclcpp::Duration::from_seconds(0.0)){}
    rclcpp::Duration time;
    rclcpp::Duration scaled_time;
  };

  ScaledFjtController::TimeData td_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle_;
  std::mutex mtx_;
  std::mutex speed_ovr_mtx_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr scaled_time_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr execution_ratio_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr unscaled_joint_target_pub_;

  bool sort_trajectory(const std::vector<std::string>& joint_names, const trajectory_msgs::msg::JointTrajectory& trj, trajectory_msgs::msg::JointTrajectory& sorted_trj);
private:
  std::vector<std::string> joint_names_;

  std::string printCurrentPos();
};
}  

#endif
