#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

static double wrap_to_pi(double a){
  while(a > M_PI) a -= 2*M_PI;
  while(a < -M_PI) a += 2*M_PI;
  return a;
}

class GoToPoseNode : public rclcpp::Node {
public:
  GoToPoseNode(): Node("goto_pose_node") {
    // --- params ---
    declare_parameter("x_goal", 2.0);
    declare_parameter("y_goal", 1.0);
    declare_parameter("theta_goal", 1.57);

    // gains / limits
    declare_parameter("k_rho",   1.0);   // drive gain (step 2)
    declare_parameter("k_align", 2.5);   // rotate-in-place gain (steps 1 & 3)
    declare_parameter("k_track", 1.2);   // small heading correction during drive

    declare_parameter("v_max", 0.6);
    declare_parameter("w_max", 1.8);

    // tolerances
    declare_parameter("pos_tol",   0.05);  // 5 cm
    declare_parameter("yaw_tol",   0.10);  // 0.1 rad
    declare_parameter("align_tol", 0.15);  // ~9 deg to finish step 1

    // topics
    declare_parameter("odom_topic", "/odom");
    declare_parameter("cmd_vel_topic", "/cmd_vel");

    // load
    xg_      = get_parameter("x_goal").as_double();
    yg_      = get_parameter("y_goal").as_double();
    thetag_  = get_parameter("theta_goal").as_double();
    k_rho_   = get_parameter("k_rho").as_double();
    k_align_ = get_parameter("k_align").as_double();
    k_track_ = get_parameter("k_track").as_double();
    v_max_   = get_parameter("v_max").as_double();
    w_max_   = get_parameter("w_max").as_double();
    pos_tol_ = get_parameter("pos_tol").as_double();
    yaw_tol_ = get_parameter("yaw_tol").as_double();
    align_tol_ = get_parameter("align_tol").as_double();
    odom_topic_ = get_parameter("odom_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 20, std::bind(&GoToPoseNode::odom_cb, this, std::placeholders::_1));

    timer_ = create_wall_timer(40ms, std::bind(&GoToPoseNode::loop, this));
    RCLCPP_INFO(get_logger(), "Staged controller: rotate -> drive -> rotate. Goal=(%.2f,%.2f,%.2f)",
                xg_, yg_, thetag_);
  }

private:
  enum Phase { ALIGN_TO_GOAL_HEADING=0, DRIVE_TO_GOAL=1, ALIGN_TO_FINAL_YAW=2, DONE=3 };

  void odom_cb(const nav_msgs::msg::Odometry & msg){
    x_ = msg.pose.pose.position.x;
    y_ = msg.pose.pose.position.y;
    const auto &q = msg.pose.pose.orientation;
    double r,p,yaw;
    tf2::Quaternion qq(q.x,q.y,q.z,q.w);
    tf2::Matrix3x3(qq).getRPY(r,p,yaw);
    th_ = yaw;
    have_odom_ = true;
  }

  void loop(){
    if(!have_odom_) return;

    geometry_msgs::msg::Twist cmd{};
    double dx = xg_ - x_;
    double dy = yg_ - y_;
    double rho = std::hypot(dx, dy);
    double bearing = std::atan2(dy, dx);      // direction to the goal
    double alpha   = wrap_to_pi(bearing - th_); // heading error towards goal
    double yaw_err = wrap_to_pi(thetag_ - th_);

    switch(phase_){
      case ALIGN_TO_GOAL_HEADING:
        // rotate in place until roughly facing the goal
        if (std::abs(alpha) > align_tol_ && rho > pos_tol_) {
          cmd.linear.x = 0.0;
          cmd.angular.z = std::clamp(k_align_ * alpha, -w_max_, w_max_);
        } else {
          phase_ = DRIVE_TO_GOAL;
          RCLCPP_INFO(get_logger(), "Aligned to goal heading -> DRIVE");
          return; // publish nothing this tick
        }
        break;

      case DRIVE_TO_GOAL:
        if (rho > pos_tol_) {
          // drive mostly straight; small heading correction
          double v = std::clamp(k_rho_ * rho, -v_max_, v_max_);
          double w = std::clamp(k_track_ * alpha, -w_max_, w_max_);
          cmd.linear.x = v;
          cmd.angular.z = w;
        } else {
          phase_ = ALIGN_TO_FINAL_YAW;
          RCLCPP_INFO(get_logger(), "Reached goal position -> FINAL ROTATE");
          return;
        }
        break;

      case ALIGN_TO_FINAL_YAW:
        if (std::abs(yaw_err) > yaw_tol_) {
          cmd.linear.x = 0.0;
          cmd.angular.z = std::clamp(k_align_ * yaw_err, -w_max_, w_max_);
        } else {
          cmd.linear.x = 0.0; cmd.angular.z = 0.0;
          phase_ = DONE;
          RCLCPP_INFO(get_logger(), "Done.");
        }
        break;

      case DONE:
      default:
        cmd.linear.x = 0.0; cmd.angular.z = 0.0;
        break;
    }

    cmd_pub_->publish(cmd);
  }

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // state
  bool have_odom_{false};
  Phase phase_{ALIGN_TO_GOAL_HEADING};
  double x_{0.0}, y_{0.0}, th_{0.0};

  // params
  double xg_, yg_, thetag_;
  double k_rho_, k_align_, k_track_;
  double v_max_, w_max_;
  double pos_tol_, yaw_tol_, align_tol_;
  std::string odom_topic_, cmd_vel_topic_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToPoseNode>());
  rclcpp::shutdown();
  return 0;
}
