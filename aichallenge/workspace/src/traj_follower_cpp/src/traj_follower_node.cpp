#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <cmath>
#include <limits>
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_control_msgs::msg::AckermannControlCommand;

class TrajFollowerNode : public rclcpp::Node {
public:
  TrajFollowerNode() : Node("traj_follower") {
    wheelbase_ = declare_parameter<double>("wheelbase", 1.0);
    l0_   = declare_parameter<double>("l0", 2.0);
    tau_  = declare_parameter<double>("tau", 0.75);
    Lmin_ = declare_parameter<double>("Lmin", 1.0);
    Lmax_ = declare_parameter<double>("Lmax", 5.0);
    near_allow_ = declare_parameter<double>("nearest_allow_dist", 50.0);
    fallback_v_ = declare_parameter<double>("fallback_speed", 8.0);

    auto qos_traj = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto qos_odom = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    traj_sub_ = create_subscription<Trajectory>(
      "/planning/scenario_planning/trajectory", qos_traj,
      [this](Trajectory::SharedPtr msg){ last_traj_ = *msg; });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", qos_odom,
      std::bind(&TrajFollowerNode::onOdom, this, std::placeholders::_1));

    ack_pub_ = create_publisher<AckermannControlCommand>(
      "/control/command/control_cmd", rclcpp::QoS(10).reliable());

    RCLCPP_INFO(get_logger(), "traj_follower_node ready.");
  }

private:
  static double yawFromQuat(const geometry_msgs::msg::Quaternion & q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void publishStop(const rclcpp::Time & stamp) {
    AckermannControlCommand cmd;
    cmd.stamp = stamp;
    cmd.lateral.steering_tire_angle = 0.0f;
    cmd.longitudinal.speed = 0.0f;
    ack_pub_->publish(cmd);
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (last_traj_.points.empty()) return;

    const auto & p = msg->pose.pose.position;
    const double yaw = yawFromQuat(msg->pose.pose.orientation);
    const double vx  = msg->twist.twist.linear.x;

    // nearest point
    size_t idx_near = 0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < last_traj_.points.size(); ++i) {
      const auto & q = last_traj_.points[i].pose.position;
      const double dx = q.x - p.x, dy = q.y - p.y;
      const double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) { best_d2 = d2; idx_near = i; }
    }
    const double d_near = std::sqrt(best_d2);
    if (d_near > near_allow_) { publishStop(msg->header.stamp); return; }

    // lookahead
    const double L = std::clamp(l0_ + tau_ * std::abs(vx), Lmin_, Lmax_);
    size_t idx_target = idx_near;
    double acc = 0.0;
    for (size_t i = idx_near; i + 1 < last_traj_.points.size(); ++i) {
      const auto & a = last_traj_.points[i].pose.position;
      const auto & b = last_traj_.points[i+1].pose.position;
      acc += std::hypot(b.x - a.x, b.y - a.y);
      if (acc >= L) { idx_target = i + 1; break; }
    }

    const auto & g = last_traj_.points[idx_target].pose.position;
    const double dx = g.x - p.x, dy = g.y - p.y;
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double xb =  c*dx + s*dy;
    const double yb = -s*dx + c*dy;
    const double Ld = std::max(1e-3, std::hypot(xb, yb));
    const double curvature = 2.0 * yb / (Ld * Ld);

    double vref = fallback_v_;
    const double vtraj = last_traj_.points[idx_near].longitudinal_velocity_mps;
    if (vtraj > 0.1) vref = vtraj;

    AckermannControlCommand cmd;
    cmd.stamp = msg->header.stamp;
    cmd.lateral.steering_tire_angle = static_cast<float>(curvature * wheelbase_);
    cmd.longitudinal.speed = static_cast<float>(vref);
    ack_pub_->publish(cmd);
  }

  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr ack_pub_;
  Trajectory last_traj_;

  double wheelbase_, l0_, tau_, Lmin_, Lmax_, near_allow_, fallback_v_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
