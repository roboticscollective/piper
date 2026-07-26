#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
enum ControlMode
{
  BOTH = 0,
  POSITION_ONLY = 1,
  ORIENTATION_ONLY = 2,
};
constexpr int kNumControlModes = 3;

// Zero all but the max-magnitude element of a 3-vector. All-zero input stays all-zero.
std::array<double, 3> dominant_axis(const std::array<double, 3> & v)
{
  size_t mi = 0;
  for (size_t i = 1; i < 3; ++i) {
    if (std::abs(v[i]) > std::abs(v[mi])) {
      mi = i;
    }
  }
  std::array<double, 3> out{0.0, 0.0, 0.0};
  if (v[mi] != 0.0) {
    out[mi] = v[mi];
  }
  return out;
}
}  // namespace

class TwistStamper : public rclcpp::Node
{
public:
  TwistStamper() : Node("twist_stamper")
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "base_link");
    deadzone_ = this->declare_parameter<double>("deadzone", 0.05);
    max_val_ = this->declare_parameter<double>("max_val", 1.0);
    smoothing_alpha_ = this->declare_parameter<double>("smoothing", 0.2);
    coast_duration_ = this->declare_parameter<double>("coast_duration", 0.2);
    dominant_axis_lock_ = this->declare_parameter<bool>("dominant_axis_lock", true);
    publish_period_ = this->declare_parameter<double>("publish_period", 0.034);

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_in", 10, std::bind(&TwistStamper::twist_callback, this, std::placeholders::_1));

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy_in", 10, std::bind(&TwistStamper::joy_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist_out", 10);

    pause_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/pause_servo");
    start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/start_servo");

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(publish_period_),
      std::bind(&TwistStamper::tick, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Twist Stamper initialized. Frame: %s | deadzone=%.3f smoothing=%.2f coast=%.2fs "
      "dominant_axis_lock=%s",
      frame_id_.c_str(), deadzone_, smoothing_alpha_, coast_duration_,
      dominant_axis_lock_ ? "true" : "false");
  }

private:
  double clamp_deadzone(double v) const
  {
    if (std::abs(v) < deadzone_) {return 0.0;}
    return std::clamp(v, -max_val_, max_val_);
  }

  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::array<double, 3> lin{
      clamp_deadzone(msg->linear.x), clamp_deadzone(msg->linear.y), clamp_deadzone(msg->linear.z)};
    std::array<double, 3> ang{
      clamp_deadzone(msg->angular.x), clamp_deadzone(msg->angular.y),
      clamp_deadzone(msg->angular.z)};

    if (dominant_axis_lock_) {
      lin = dominant_axis(lin);
      ang = dominant_axis(ang);
    }

    raw_lin_ = lin;
    raw_ang_ = ang;
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::vector<int32_t> buttons = msg->buttons;
    while (buttons.size() < 2) {buttons.push_back(0);}
    while (prev_buttons_.size() < 2) {prev_buttons_.push_back(0);}

    // Button 0 (left), rising edge: cycle BOTH -> POSITION_ONLY -> ORIENTATION_ONLY
    if (buttons[0] && !prev_buttons_[0]) {
      control_mode_ = (control_mode_ + 1) % kNumControlModes;
      static const char * kModeNames[] = {"BOTH", "POSITION_ONLY", "ORIENTATION_ONLY"};
      RCLCPP_INFO(this->get_logger(), "Control mode -> %s", kModeNames[control_mode_]);
    }

    // Button 1 (right), rising edge: toggle Servo pause/resume.
    // (No "resync" equivalent here -- Servo streams velocity off live robot state and
    // holds no accumulated target pose to drift, unlike an absolute-pose impedance
    // controller, so there is nothing to resync.)
    if (buttons[1] && !prev_buttons_[1]) {
      toggle_pause();
    }

    prev_buttons_ = buttons;
  }

  void toggle_pause()
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    if (!servo_paused_) {
      if (!pause_client_->wait_for_service(std::chrono::milliseconds(200))) {
        RCLCPP_WARN(this->get_logger(), "[BTN1] pause_servo service unavailable");
        return;
      }
      pause_client_->async_send_request(request);
      servo_paused_ = true;
      RCLCPP_INFO(this->get_logger(), "[BTN1] Servo paused");
    } else {
      if (!start_client_->wait_for_service(std::chrono::milliseconds(200))) {
        RCLCPP_WARN(this->get_logger(), "[BTN1] start_servo service unavailable");
        return;
      }
      start_client_->async_send_request(request);
      servo_paused_ = false;
      RCLCPP_INFO(this->get_logger(), "[BTN1] Servo resumed");
    }
  }

  void tick()
  {
    const rclcpp::Time now = this->now();

    const bool has_input =
      std::any_of(raw_lin_.begin(), raw_lin_.end(), [](double v) {return v != 0.0;}) ||
      std::any_of(raw_ang_.begin(), raw_ang_.end(), [](double v) {return v != 0.0;});

    if (has_input) {
      last_input_time_ = now;
      have_last_input_ = true;
    }

    const bool in_coast = have_last_input_ &&
      (now - last_input_time_).seconds() < coast_duration_;

    if (!(has_input || in_coast)) {
      return;
    }

    for (size_t i = 0; i < 3; ++i) {
      if (raw_lin_[i] != 0.0) {
        smooth_lin_[i] += smoothing_alpha_ * (raw_lin_[i] - smooth_lin_[i]);
      } else {
        smooth_lin_[i] = 0.0;
      }
      if (raw_ang_[i] != 0.0) {
        smooth_ang_[i] += smoothing_alpha_ * (raw_ang_[i] - smooth_ang_[i]);
      } else {
        smooth_ang_[i] = 0.0;
      }
    }

    auto msg_out = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg_out->header.stamp = now;
    msg_out->header.frame_id = frame_id_;

    const bool publish_lin = control_mode_ != ORIENTATION_ONLY;
    const bool publish_ang = control_mode_ != POSITION_ONLY;

    msg_out->twist.linear.x = publish_lin ? smooth_lin_[0] : 0.0;
    msg_out->twist.linear.y = publish_lin ? smooth_lin_[1] : 0.0;
    msg_out->twist.linear.z = publish_lin ? smooth_lin_[2] : 0.0;
    msg_out->twist.angular.x = publish_ang ? smooth_ang_[0] : 0.0;
    msg_out->twist.angular.y = publish_ang ? smooth_ang_[1] : 0.0;
    msg_out->twist.angular.z = publish_ang ? smooth_ang_[2] : 0.0;

    pub_->publish(std::move(msg_out));
  }

  // Parameters
  std::string frame_id_;
  double deadzone_;
  double max_val_;
  double smoothing_alpha_;
  double coast_duration_;
  bool dominant_axis_lock_;
  double publish_period_;

  // State
  std::array<double, 3> raw_lin_{0.0, 0.0, 0.0};
  std::array<double, 3> raw_ang_{0.0, 0.0, 0.0};
  std::array<double, 3> smooth_lin_{0.0, 0.0, 0.0};
  std::array<double, 3> smooth_ang_{0.0, 0.0, 0.0};
  rclcpp::Time last_input_time_;
  bool have_last_input_{false};
  int control_mode_{BOTH};
  std::vector<int32_t> prev_buttons_;
  bool servo_paused_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistStamper>());
  rclcpp::shutdown();
  return 0;
}
