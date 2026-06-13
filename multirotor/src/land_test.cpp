#include <cmath>
#include <iostream>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace {

float clamp_symmetric(float value, float limit) {
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

float clamp_range(float value, float min_value, float max_value) {
  if (value > max_value) return max_value;
  if (value < min_value) return min_value;
  return value;
}

}  // namespace


class LandingTest : public rclcpp::Node {
public:
  LandingTest() : Node("landing") {
    odom_sub_ = this->create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry",
      rclcpp::SensorDataQoS(),
      [this](const VehicleOdometry::SharedPtr msg) {
        curr_odom_ = *msg;
        has_odom_ = true;
      });

    landed_sub_ = this->create_subscription<VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected",
      rclcpp::SensorDataQoS(),
      [this](const VehicleLandDetected::SharedPtr msg) {
        landed_ = msg->landed;
      });

    desired_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/landing/coordinates",
      10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        desired_x_ = msg->point.x;   // right(+), [m]
        desired_y_ = msg->point.y;   // forward(+), [m]
        acc_alt_ = -msg->point.z;    // existing convention
      });

    declare_parameters();

    offboard_control_mode_publisher_ =
      this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

    trajectory_setpoint_publisher_ =
      this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

    vehicle_command_publisher_ =
      this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    mission_mode_publisher_ =
      this->create_publisher<std_msgs::msg::String>("/mission_mode", 10);

    timer_ = this->create_wall_timer(100ms, [this]() { timer_callback(); });
  }

private:
  enum Mission {
    FLIGHT,
    LANDING,
    FINISHED,
  };

  enum LandingMode {
    POSITION_XY_VELOCITY_Z = 0,
    VELOCITY_XYZ = 1,
  };

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_mode_publisher_;

  rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr landed_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr desired_setpoint_sub_;

  // PX4 / mission state
  VehicleOdometry curr_odom_{};

  bool has_odom_ = false;
  bool landed_ = false;
  bool disarm_sent_ = false;
  bool arm_requested_ = false;
  bool offboard_requested_ = false;
  bool nav_land_sent_ = false;

  int preflight_setpoint_count_ = 0;
  int offboard_setpoint_counter_ = 0;

  Mission mission_mode_ = FLIGHT;

  // Target state from vision node
  float desired_x_ = 0.0f;  // right(+), [m]
  float desired_y_ = 0.0f;  // forward(+), [m]
  float acc_alt_ = 0.0f;

  int lost_count_ = 0;
  int hold_counter_ = 0;

  // Parameters
  int start_mode_ = 0;
  int land_mode_ = VELOCITY_XYZ;

  float start_x_ = 0.0f;
  float start_y_ = 0.0f;
  float start_z_ = 0.0f;

  int lost_abort_ = 700;
  int align_need_ = 5;

  float max_xy_ = 0.6f;
  float tol_m_ = 0.8f;
  float deadband_m_ = 0.05f;

  float tanh_min_xy_ = 0.05f;
  float tanh_gain_ = 1.2f;

  float atan_position_gain_ = 1.2f;
  float position_step_max_m_ = 0.50f;
  float position_step_min_m_ = 0.05f;

  float descent_high_mps_ = 0.40f;
  float descent_mid_mps_ = 0.30f;
  float descent_low_mps_ = 0.20f;

  float low_enough_ = -0.7f;

  bool use_q_inverse_ = false;

  const float nan_ = std::numeric_limits<float>::quiet_NaN();

  // Methods
  void declare_parameters();
  void read_parameters();
  void timer_callback();

  void arm();
  void disarm();

  void publish_offboard_control_mode();
  void publish_trajectory_setpoint();
  void publish_vehicle_command(
    uint16_t command,
    float param1 = 0.0f,
    float param2 = 0.0f,
    float param3 = 0.0f,
    float param4 = 0.0f);

  void land();

  float select_descent_speed(float alt_m, bool valid_xy) const;
  Eigen::Quaternionf current_attitude_quaternion() const;
  Eigen::Vector3f body_frd_to_ned(const Eigen::Vector3f &body_frd) const;
};


void LandingTest::declare_parameters() {
  // 0: x/y position setpoint + z velocity setpoint
  // 1: x/y/z velocity setpoint
  this->declare_parameter<int>("land_param", VELOCITY_XYZ);

  // 0: start landing immediately
  // 1: fly to start_x/y/z first, then landing
  this->declare_parameter<int>("start_param", 0);

  this->declare_parameter<float>("start_x_param", 0.0f);
  this->declare_parameter<float>("start_y_param", 0.0f);
  this->declare_parameter<float>("start_z_param", 0.0f);

  this->declare_parameter<int>("lost_abort_", 700);
  this->declare_parameter<float>("max_xy_", 0.4f);
  this->declare_parameter<float>("tol_m_", 0.8f);
  this->declare_parameter<int>("align_need_", 5);

  this->declare_parameter<float>("deadband_m_", 0.05f);

  // Velocity-based x/y controller.
  this->declare_parameter<float>("tanh_min_xy_", 0.10f);
  this->declare_parameter<float>("tanh_gain_", 1.2f);

  // Position-based x/y controller.
  this->declare_parameter<float>("atan_position_gain_", 1.2f);
  this->declare_parameter<float>("position_step_max_m_", 0.50f);
  this->declare_parameter<float>("position_step_min_m_", 0.05f);

  // z is always velocity-based.
  this->declare_parameter<float>("descent_high_mps_", 0.40f);
  this->declare_parameter<float>("descent_mid_mps_", 0.30f);
  this->declare_parameter<float>("descent_low_mps_", 0.20f);
}


void LandingTest::read_parameters() {
  land_mode_ = this->get_parameter("land_param").as_int();
  start_mode_ = this->get_parameter("start_param").as_int();

  start_x_ = static_cast<float>(this->get_parameter("start_x_param").as_double());
  start_y_ = static_cast<float>(this->get_parameter("start_y_param").as_double());
  start_z_ = -static_cast<float>(this->get_parameter("start_z_param").as_double());

  lost_abort_ = this->get_parameter("lost_abort_").as_int();
  align_need_ = this->get_parameter("align_need_").as_int();

  max_xy_ = static_cast<float>(this->get_parameter("max_xy_").as_double());
  tol_m_ = static_cast<float>(this->get_parameter("tol_m_").as_double());
  deadband_m_ = static_cast<float>(this->get_parameter("deadband_m_").as_double());

  tanh_min_xy_ = static_cast<float>(this->get_parameter("tanh_min_xy_").as_double());
  tanh_gain_ = static_cast<float>(this->get_parameter("tanh_gain_").as_double());

  atan_position_gain_ =
    static_cast<float>(this->get_parameter("atan_position_gain_").as_double());

  position_step_max_m_ =
    static_cast<float>(this->get_parameter("position_step_max_m_").as_double());

  position_step_min_m_ =
    static_cast<float>(this->get_parameter("position_step_min_m_").as_double());

  descent_high_mps_ =
    static_cast<float>(this->get_parameter("descent_high_mps_").as_double());

  descent_mid_mps_ =
    static_cast<float>(this->get_parameter("descent_mid_mps_").as_double());

  descent_low_mps_ =
    static_cast<float>(this->get_parameter("descent_low_mps_").as_double());
}


void LandingTest::timer_callback() {
  if (!has_odom_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
    return;
  }

  read_parameters();

  if (start_mode_ == 0 && mission_mode_ == FLIGHT) {
    mission_mode_ = LANDING;
  }

  publish_offboard_control_mode();

  std_msgs::msg::String mission_msg;

  switch (mission_mode_) {
    case FLIGHT:
      publish_trajectory_setpoint();
      mission_msg.data = "FLIGHT";
      break;

    case LANDING:
      land();
      mission_msg.data = "LANDING";
      break;

    case FINISHED:
    default:
      if (landed_ && !disarm_sent_) {
        disarm();
        disarm_sent_ = true;
      }

      mission_msg.data = "FINISHED";
      mission_mode_publisher_->publish(mission_msg);
      return;
  }

  mission_mode_publisher_->publish(mission_msg);

  // PX4 Offboard requires a short setpoint stream before mode switch.
  if (!offboard_requested_) {
    preflight_setpoint_count_++;

    if (preflight_setpoint_count_ > 20) {
      RCLCPP_INFO(this->get_logger(), "Requesting OFFBOARD mode");
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      offboard_requested_ = true;
    }

    return;
  }

  if (!arm_requested_) {
    RCLCPP_INFO(this->get_logger(), "Requesting ARM");
    arm();
    arm_requested_ = true;
    return;
  }

  offboard_setpoint_counter_++;
}


void LandingTest::arm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
  RCLCPP_INFO(this->get_logger(), "Arm command send");
}


void LandingTest::disarm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
  RCLCPP_INFO(this->get_logger(), "Disarm command send");
}


void LandingTest::publish_offboard_control_mode() {
  OffboardControlMode msg{};

  if (mission_mode_ == LANDING) {
    if (land_mode_ == POSITION_XY_VELOCITY_Z) {
      msg.position = true;   // x/y
      msg.velocity = true;   // z
    } else {
      msg.position = false;
      msg.velocity = true;   // x/y/z
    }
  } else {
    msg.position = true;
    msg.velocity = false;
  }

  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

  offboard_control_mode_publisher_->publish(msg);
}


void LandingTest::publish_trajectory_setpoint() {
  if (curr_odom_.timestamp == 0) {
    RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
    return;
  }

  TrajectorySetpoint msg{};

  Eigen::Vector3f current(
    curr_odom_.position[0],
    curr_odom_.position[1],
    curr_odom_.position[2]);

  Eigen::Vector3f target(start_x_, start_y_, start_z_);
  Eigen::Vector3f to_wp = target - current;
  const float dist = to_wp.norm();

  msg.position = {target[0], target[1], target[2]};

  if (dist < 3.0f) {
    hold_counter_++;

    if (hold_counter_ > 20) {
      hold_counter_ = 0;
      mission_mode_ = LANDING;
      RCLCPP_INFO(this->get_logger(), "[LANDING] Initiating landing sequence");
      return;
    }
  } else {
    hold_counter_ = 0;
  }

  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}


float LandingTest::select_descent_speed(float alt_m, bool valid_xy) const {
  if (!valid_xy || hold_counter_ < align_need_) {
    return 0.0f;
  }

  if (alt_m > 2.0f) {
    return descent_high_mps_;
  }

  if (alt_m > 0.8f) {
    return descent_mid_mps_;
  }

  return descent_low_mps_;
}


Eigen::Quaternionf LandingTest::current_attitude_quaternion() const {
  Eigen::Quaternionf q(
    curr_odom_.q[0],
    curr_odom_.q[1],
    curr_odom_.q[2],
    curr_odom_.q[3]);

  q.normalize();
  return q;
}


Eigen::Vector3f LandingTest::body_frd_to_ned(const Eigen::Vector3f &body_frd) const {
  const Eigen::Quaternionf q = current_attitude_quaternion();

  if (use_q_inverse_) {
    return q.conjugate() * body_frd;
  }

  return q * body_frd;
}


void LandingTest::land() {
  TrajectorySetpoint msg{};

  const float alt_m = -acc_alt_;

  const bool valid_xy =
    std::isfinite(desired_x_) &&
    std::isfinite(desired_y_);

  const bool aligned =
    valid_xy &&
    std::fabs(desired_x_) < tol_m_ &&
    std::fabs(desired_y_) < tol_m_;

  if (!valid_xy) {
    lost_count_++;
    hold_counter_ = 0;
  } else {
    lost_count_ = 0;
    hold_counter_ = aligned ? hold_counter_ + 1 : 0;
  }

  if (lost_count_ > lost_abort_) {
    RCLCPP_WARN(
      this->get_logger(),
      "[LANDING] target lost too long -> switch PX4 to POSITION mode");

    publish_vehicle_command(
      VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0f,
      3.0f);

    mission_mode_ = FINISHED;
    return;
  }

  const float ex = valid_xy ? desired_x_ : 0.0f;  // right(+)
  const float ey = valid_xy ? desired_y_ : 0.0f;  // forward(+)
  const float err_dist = std::sqrt(ex * ex + ey * ey);

  const float descent_mps = select_descent_speed(alt_m, valid_xy);

  if (land_mode_ == POSITION_XY_VELOCITY_Z) {
    Eigen::Vector3f current_ned(
      curr_odom_.position[0],
      curr_odom_.position[1],
      curr_odom_.position[2]);

    float xy_step = 0.0f;
    Eigen::Vector3f target_body_frd(0.0f, 0.0f, 0.0f);

    if (valid_xy && err_dist >= deadband_m_) {
      const float ux = ex / err_dist;  // right ratio
      const float uy = ey / err_dist;  // forward ratio

      xy_step =
        position_step_max_m_ *
        (2.0f / static_cast<float>(M_PI)) *
        std::atan(atan_position_gain_ * err_dist);

      xy_step = clamp_range(xy_step, position_step_min_m_, position_step_max_m_);

      const float step_right = xy_step * ux;
      const float step_forward = xy_step * uy;

      // BODY/FRD: x = forward, y = right, z = down
      target_body_frd = Eigen::Vector3f(step_forward, step_right, 0.0f);
    }

    Eigen::Vector3f target_ned = current_ned + body_frd_to_ned(target_body_frd);

    // x/y position-based, z velocity-based.
    msg.position = {target_ned[0], target_ned[1], nan_};
    msg.velocity = {nan_, nan_, descent_mps};

    RCLCPP_INFO(
      this->get_logger(),
      "[POS_XY_VEL_Z] valid=%d aligned=%d hold=%d/%d dx=%.3f dy=%.3f err=%.3f tol=%.3f xy_step=%.3f vz=%.3f alt=%.3f",
      valid_xy,
      aligned,
      hold_counter_,
      align_need_,
      desired_x_,
      desired_y_,
      err_dist,
      tol_m_,
      xy_step,
      descent_mps,
      alt_m);
  } else {
    float v_forward = 0.0f;
    float v_right = 0.0f;
    float v_close = 0.0f;

    if (valid_xy && err_dist >= deadband_m_) {
      const float ux = ex / err_dist;  // right ratio
      const float uy = ey / err_dist;  // forward ratio

      v_close = max_xy_ * std::tanh(tanh_gain_ * err_dist);
      v_close = clamp_range(v_close, tanh_min_xy_, max_xy_);

      v_right = clamp_symmetric(v_close * ux, max_xy_);
      v_forward = clamp_symmetric(v_close * uy, max_xy_);
    }

    Eigen::Vector3f v_body(v_forward, v_right, 0.0f);
    Eigen::Vector3f v_ned = body_frd_to_ned(v_body);

    // x/y/z velocity-based.
    msg.position = {nan_, nan_, nan_};
    msg.velocity = {v_ned[0], v_ned[1], descent_mps};

    RCLCPP_INFO(
      this->get_logger(),
      "[VEL_XYZ] dx=%.3f dy=%.3f err=%.3f tol=%.3f v_close=%.3f vz=%.3f alt=%.3f",
      desired_x_,
      desired_y_,
      err_dist,
      tol_m_,
      v_close,
      descent_mps,
      alt_m);
  }

  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);

  if (
    valid_xy &&
    hold_counter_ >= align_need_ &&
    acc_alt_ > low_enough_ &&
    !nav_land_sent_) {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);

    RCLCPP_INFO(
      this->get_logger(),
      "[LANDING] aligned & low enough (alt=%.2f m). NAV_LAND.",
      alt_m);

    nav_land_sent_ = true;
    mission_mode_ = FINISHED;
  }
}


void LandingTest::publish_vehicle_command(
  uint16_t command,
  float param1,
  float param2,
  float param3,
  float param4) {
  VehicleCommand msg{};

  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.param4 = param4;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.target_system = 1;
  msg.command = command;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

  vehicle_command_publisher_->publish(msg);
}


int main(int argc, char *argv[]) {
  std::cout << "Starting landing test" << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandingTest>());
  rclcpp::shutdown();

  return 0;
}
