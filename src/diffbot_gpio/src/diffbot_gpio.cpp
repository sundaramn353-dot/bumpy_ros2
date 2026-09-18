// diffbot_gpio.cpp  — Standalone GPIO motor-driver / encoder node
// No ros2_control dependency. Publishes sensor_msgs/JointState and
// subscribes to std_msgs/Float64MultiArray velocity commands.

#include "diffbot_gpio/diffbot_gpio.hpp"

#include <pigpiod_if2.h>
#include <cmath>
#include <algorithm>
#include <chrono>

namespace diffbot_gpio
{

// ─── Pin Map ────────────────────────────────────────────────────────────────
static constexpr int LEFT_IN1  = 5;
static constexpr int LEFT_IN2  = 6;
static constexpr int LEFT_PWM  = 13;

static constexpr int RIGHT_IN1 = 8;
static constexpr int RIGHT_IN2 = 7;
static constexpr int RIGHT_PWM = 12;

// Encoder Pins
static constexpr int ENC_LEFT_C1  = 17;
static constexpr int ENC_LEFT_C2  = 27;

static constexpr int ENC_RIGHT_C1 = 23;
static constexpr int ENC_RIGHT_C2 = 24;

// PWM settings
static constexpr int PWM_RANGE = 255;
static constexpr int PWM_FREQ  = 20000;  // Hz

// Quadrature Encoder Look-up Matrix
static constexpr int QEM[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};

// ─── Constructor ─────────────────────────────────────────────────────────────
DiffBotGPIONode::DiffBotGPIONode()
: Node("diffbot_gpio_node")
{
  // ── Declare Parameters ──────────────────────────────────────────────────
  this->declare_parameter("pulses_per_rotation", 6090.0);   // calibrated: 1400 × 4.35
  this->declare_parameter("wheel_radius",        0.022);    // metres  (44 mm diam / 2)
  this->declare_parameter("wheel_separation",    0.0829);   // metres — calibrated 2026-09 (was 0.080)
  this->declare_parameter("update_rate_hz",      50.0);     // control-loop Hz
  // Flip encoder direction if forward motion produces negative ticks.
  // Both set true here based on calibration (odom was -4.35 m for +1 m physical).
  this->declare_parameter("left_encoder_reversed",  true);
  this->declare_parameter("right_encoder_reversed", true);

  // PID gains
  this->declare_parameter("left_kp",          30.0);
  this->declare_parameter("left_ki",          10.0);
  this->declare_parameter("left_kd",           0.0);
  this->declare_parameter("left_offset_fwd",   0.0);
  this->declare_parameter("left_offset_bwd",   0.0);

  this->declare_parameter("right_kp",         30.0);
  this->declare_parameter("right_ki",         10.0);
  this->declare_parameter("right_kd",          0.0);
  this->declare_parameter("right_offset_fwd",  0.0);
  this->declare_parameter("right_offset_bwd",  0.0);

  load_params();

  // ── Parameter-change callback ────────────────────────────────────────────
  param_cb_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params)
    {
      std::lock_guard<std::mutex> lock(pid_mutex_);
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        if      (p.get_name() == "left_kp")          left_pid_.kp           = p.as_double();
        else if (p.get_name() == "left_ki")          left_pid_.ki           = p.as_double();
        else if (p.get_name() == "left_kd")          left_pid_.kd           = p.as_double();
        else if (p.get_name() == "left_offset_fwd")  left_pid_.offset_fwd   = p.as_double();
        else if (p.get_name() == "left_offset_bwd")  left_pid_.offset_bwd   = p.as_double();
        else if (p.get_name() == "right_kp")         right_pid_.kp          = p.as_double();
        else if (p.get_name() == "right_ki")         right_pid_.ki          = p.as_double();
        else if (p.get_name() == "right_kd")         right_pid_.kd          = p.as_double();
        else if (p.get_name() == "right_offset_fwd") right_pid_.offset_fwd  = p.as_double();
        else if (p.get_name() == "right_offset_bwd") right_pid_.offset_bwd  = p.as_double();
        else if (p.get_name() == "pulses_per_rotation") pulses_per_rotation_ = p.as_double();
        else if (p.get_name() == "left_encoder_reversed")
          left_enc_sign_  = p.as_bool() ? -1 : 1;
        else if (p.get_name() == "right_encoder_reversed")
          right_enc_sign_ = p.as_bool() ? -1 : 1;
      }
      return result;
    });

  // ── ROS Interfaces ───────────────────────────────────────────────────────
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SensorDataQoS());

  // Velocity commands: [left_vel_rad_s, right_vel_rad_s]
  cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "simple_velocity_controller/commands",
    10,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if (msg->data.size() >= 2) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        left_cmd_  = msg->data[0];
        right_cmd_ = msg->data[1];
      }
    });

  // ── pigpio ───────────────────────────────────────────────────────────────
  pigpio_handle_ = pigpio_start(nullptr, nullptr);
  if (pigpio_handle_ < 0) {
    RCLCPP_ERROR(get_logger(),
      "Failed to connect to pigpiod! Run: sudo systemctl start pigpiod");
  } else {
    RCLCPP_INFO(get_logger(), "Connected to pigpiod.");
    configure_gpio();
  }

  // ── Control loop timer ───────────────────────────────────────────────────
  double hz = this->get_parameter("update_rate_hz").as_double();
  auto period = std::chrono::duration<double>(1.0 / hz);
  loop_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&DiffBotGPIONode::control_loop, this));

  RCLCPP_INFO(get_logger(),
    "DiffBotGPIONode ready. PPR=%.0f  R=%.4f m  Sep=%.4f m  Loop=%.0f Hz",
    pulses_per_rotation_, wheel_radius_, wheel_separation_, hz);
}

// ─── Destructor ───────────────────────────────────────────────────────────────
DiffBotGPIONode::~DiffBotGPIONode()
{
  if (pigpio_handle_ >= 0) {
    set_PWM_dutycycle(pigpio_handle_, LEFT_PWM,  0);
    set_PWM_dutycycle(pigpio_handle_, RIGHT_PWM, 0);
    pigpio_stop(pigpio_handle_);
  }
}

// ─── Load / reload parameters ────────────────────────────────────────────────
void DiffBotGPIONode::load_params()
{
  pulses_per_rotation_ = this->get_parameter("pulses_per_rotation").as_double();
  wheel_radius_        = this->get_parameter("wheel_radius").as_double();
  wheel_separation_    = this->get_parameter("wheel_separation").as_double();

  left_pid_.kp         = this->get_parameter("left_kp").as_double();
  left_pid_.ki         = this->get_parameter("left_ki").as_double();
  left_pid_.kd         = this->get_parameter("left_kd").as_double();
  left_pid_.offset_fwd = this->get_parameter("left_offset_fwd").as_double();
  left_pid_.offset_bwd = this->get_parameter("left_offset_bwd").as_double();

  right_pid_.kp         = this->get_parameter("right_kp").as_double();
  right_pid_.ki         = this->get_parameter("right_ki").as_double();
  right_pid_.kd         = this->get_parameter("right_kd").as_double();
  right_pid_.offset_fwd = this->get_parameter("right_offset_fwd").as_double();
  right_pid_.offset_bwd = this->get_parameter("right_offset_bwd").as_double();

  // Encoder direction: -1 = reversed, +1 = normal
  left_enc_sign_  = this->get_parameter("left_encoder_reversed").as_bool()  ? -1 : 1;
  right_enc_sign_ = this->get_parameter("right_encoder_reversed").as_bool() ? -1 : 1;

  RCLCPP_INFO(get_logger(),
    "Params: PPR=%.0f  Lsign=%d  Rsign=%d  PID_L(%.1f,%.1f,%.1f)  PID_R(%.1f,%.1f,%.1f)",
    pulses_per_rotation_, left_enc_sign_, right_enc_sign_,
    left_pid_.kp, left_pid_.ki, left_pid_.kd,
    right_pid_.kp, right_pid_.ki, right_pid_.kd);
}

// ─── GPIO Configuration ──────────────────────────────────────────────────────
void DiffBotGPIONode::configure_gpio()
{
  if (pigpio_handle_ < 0) return;

  // Motor output pins
  set_mode(pigpio_handle_, LEFT_IN1,  PI_OUTPUT);
  set_mode(pigpio_handle_, LEFT_IN2,  PI_OUTPUT);
  set_mode(pigpio_handle_, LEFT_PWM,  PI_OUTPUT);
  set_mode(pigpio_handle_, RIGHT_IN1, PI_OUTPUT);
  set_mode(pigpio_handle_, RIGHT_IN2, PI_OUTPUT);
  set_mode(pigpio_handle_, RIGHT_PWM, PI_OUTPUT);

  set_PWM_frequency(pigpio_handle_, LEFT_PWM,  PWM_FREQ);
  set_PWM_frequency(pigpio_handle_, RIGHT_PWM, PWM_FREQ);

  // Encoder input pins (pull-up)
  set_mode(pigpio_handle_, ENC_LEFT_C1,  PI_INPUT);
  set_mode(pigpio_handle_, ENC_LEFT_C2,  PI_INPUT);
  set_pull_up_down(pigpio_handle_, ENC_LEFT_C1, PI_PUD_UP);
  set_pull_up_down(pigpio_handle_, ENC_LEFT_C2, PI_PUD_UP);

  set_mode(pigpio_handle_, ENC_RIGHT_C1, PI_INPUT);
  set_mode(pigpio_handle_, ENC_RIGHT_C2, PI_INPUT);
  set_pull_up_down(pigpio_handle_, ENC_RIGHT_C1, PI_PUD_UP);
  set_pull_up_down(pigpio_handle_, ENC_RIGHT_C2, PI_PUD_UP);

  // ── Encoder callbacks ──────────────────────────────────────────────────
  auto left_cb = [](int, unsigned gpio, unsigned level, uint32_t, void * user)
  {
    auto * hw = static_cast<DiffBotGPIONode *>(user);
    if (!hw) return;
    int c1 = (gpio == ENC_LEFT_C1) ? static_cast<int>(level) : (hw->left_enc_state_ >> 1) & 1;
    int c2 = (gpio == ENC_LEFT_C2) ? static_cast<int>(level) : (hw->left_enc_state_ & 1);
    int new_state = (c1 << 1) | c2;
    // Negate: this motor's encoder runs backward for "forward" wheel direction
    hw->left_ticks_ -= hw->left_enc_sign_ * QEM[(hw->left_enc_state_ << 2) | new_state];
    hw->left_enc_state_ = new_state;
  };

  auto right_cb = [](int, unsigned gpio, unsigned level, uint32_t, void * user)
  {
    auto * hw = static_cast<DiffBotGPIONode *>(user);
    if (!hw) return;
    int c1 = (gpio == ENC_RIGHT_C1) ? static_cast<int>(level) : (hw->right_enc_state_ >> 1) & 1;
    int c2 = (gpio == ENC_RIGHT_C2) ? static_cast<int>(level) : (hw->right_enc_state_ & 1);
    int new_state = (c1 << 1) | c2;
    hw->right_ticks_ += hw->right_enc_sign_ * QEM[(hw->right_enc_state_ << 2) | new_state];
    hw->right_enc_state_ = new_state;
  };

  callback_ex(pigpio_handle_, ENC_LEFT_C1,  EITHER_EDGE, left_cb,  this);
  callback_ex(pigpio_handle_, ENC_LEFT_C2,  EITHER_EDGE, left_cb,  this);
  callback_ex(pigpio_handle_, ENC_RIGHT_C1, EITHER_EDGE, right_cb, this);
  callback_ex(pigpio_handle_, ENC_RIGHT_C2, EITHER_EDGE, right_cb, this);

  // Seed current state
  left_enc_state_  = (gpio_read(pigpio_handle_, ENC_LEFT_C1)  << 1) | gpio_read(pigpio_handle_, ENC_LEFT_C2);
  right_enc_state_ = (gpio_read(pigpio_handle_, ENC_RIGHT_C1) << 1) | gpio_read(pigpio_handle_, ENC_RIGHT_C2);

  RCLCPP_INFO(get_logger(), "GPIO configured.");
}

// ─── Main control loop (wall-timer callback) ──────────────────────────────────
void DiffBotGPIONode::control_loop()
{
  // --- Attempt reconnection if pigpio lost ---
  if (pigpio_handle_ < 0) {
    if (++retry_counter_ % 50 == 0) {
      pigpio_handle_ = pigpio_start(nullptr, nullptr);
      if (pigpio_handle_ >= 0) {
        RCLCPP_INFO(get_logger(), "Reconnected to pigpiod.");
        configure_gpio();
        retry_counter_ = 0;
      }
    }
    return;
  }

  const auto now    = this->get_clock()->now();
  const double dt   = (prev_time_.nanoseconds() == 0)
                      ? (1.0 / this->get_parameter("update_rate_hz").as_double())
                      : (now - prev_time_).seconds();
  prev_time_ = now;

  if (dt <= 0.0) return;

  // ── Read encoder ticks (atomic swap) ────────────────────────────────────
  const int lt = left_ticks_.exchange(0);
  const int rt = right_ticks_.exchange(0);

  const double rad_per_tick = (2.0 * M_PI) / pulses_per_rotation_;

  // NOTE: raw tick counting (see left_cb/right_cb above) produces
  // forward = NEGATIVE ticks. Targets sent over
  // simple_velocity_controller/commands (left_cmd_/right_cmd_) come from
  // simple_controller's un-corrected kinematic matrix, which assumes
  // forward = POSITIVE. Negating here makes this node's own PID feedback
  // (left_vel_meas_/right_vel_meas_) match the sign convention of the
  // targets it's being compared against, and makes left_pos_/right_pos_
  // (published as joint_states) forward = POSITIVE, the standard
  // convention. simple_controller.py's odometry callback must NOT also
  // negate these deltas anymore — see the corresponding fix there.
  const double dl = -(lt * rad_per_tick);  // delta position left  [rad]
  const double dr = -(rt * rad_per_tick);  // delta position right [rad]

  left_pos_  += dl;
  right_pos_ += dr;

  const double left_vel_meas  = dl / dt;   // rad/s
  const double right_vel_meas = dr / dt;   // rad/s

  // ── Publish JointState ───────────────────────────────────────────────────
  sensor_msgs::msg::JointState js;
  js.header.stamp.sec     = static_cast<int32_t>(now.nanoseconds() / 1'000'000'000LL);
  js.header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1'000'000'000LL);
  js.name            = {"left_wheel_joint", "right_wheel_joint"};
  js.position        = {left_pos_,       right_pos_};
  js.velocity        = {left_vel_meas,   right_vel_meas};
  joint_state_pub_->publish(js);

  // ── PID → Motor drive ────────────────────────────────────────────────────
  double left_cmd, right_cmd;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    left_cmd  = left_cmd_;
    right_cmd = right_cmd_;
  }

  auto compute_pid = [&](PID & pid, double target, double measured) -> double
  {
    if (std::abs(target) < 0.001) {
      pid.integral    = 0.0;
      pid.prev_error  = 0.0;
      return 0.0;
    }
    double error    = target - measured;
    pid.integral   += error * dt;
    // Anti-windup: clamp integral contribution to ±100
    pid.integral    = std::clamp(pid.integral, -100.0 / std::max(pid.ki, 1e-9),
                                               100.0  / std::max(pid.ki, 1e-9));
    double deriv    = (error - pid.prev_error) / dt;
    pid.prev_error  = error;
    double out      = pid.kp * error + pid.ki * pid.integral + pid.kd * deriv;
    return std::clamp(out, pid.min_output, pid.max_output);
  };

  double lpwm, rpwm;
  {
    std::lock_guard<std::mutex> lock(pid_mutex_);
    lpwm = compute_pid(left_pid_,  left_cmd,  left_vel_meas);
    rpwm = compute_pid(right_pid_, right_cmd, right_vel_meas);
    // Static friction offset
    if (lpwm >  0.001) lpwm += left_pid_.offset_fwd;
    else if (lpwm < -0.001) lpwm -= left_pid_.offset_bwd;
    if (rpwm >  0.001) rpwm += right_pid_.offset_fwd;
    else if (rpwm < -0.001) rpwm -= right_pid_.offset_bwd;
  }

  drive_motor(LEFT_IN1,  LEFT_IN2,  LEFT_PWM,  lpwm);
  drive_motor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM, rpwm);
}

// ─── Motor driver helper ──────────────────────────────────────────────────────
// pwm_value is the raw signed PWM duty (-255 … +255).
void DiffBotGPIONode::drive_motor(int in1, int in2, int pwm_pin, double pwm_value)
{
  int duty = std::clamp(static_cast<int>(std::abs(pwm_value)), 0, PWM_RANGE);

  if (pwm_value > 0.5) {
    gpio_write(pigpio_handle_, in1, 1);
    gpio_write(pigpio_handle_, in2, 0);
    set_PWM_dutycycle(pigpio_handle_, pwm_pin, duty);
  } else if (pwm_value < -0.5) {
    gpio_write(pigpio_handle_, in1, 0);
    gpio_write(pigpio_handle_, in2, 1);
    set_PWM_dutycycle(pigpio_handle_, pwm_pin, duty);
  } else {
    // Active brake: both IN high, full PWM
    gpio_write(pigpio_handle_, in1, 1);
    gpio_write(pigpio_handle_, in2, 1);
    set_PWM_dutycycle(pigpio_handle_, pwm_pin, PWM_RANGE);
  }
}

}  // namespace diffbot_gpio

// ─── main ────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<diffbot_gpio::DiffBotGPIONode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}