#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <atomic>
#include <mutex>
#include <thread>

namespace diffbot_gpio
{

class DiffBotGPIONode : public rclcpp::Node
{
public:
  DiffBotGPIONode();
  ~DiffBotGPIONode();

  // Accessible by static pigpio callbacks
  std::atomic<int> left_ticks_{0};
  std::atomic<int> right_ticks_{0};
  int left_enc_state_  = 0;
  int right_enc_state_ = 0;

private:
  // ── Setup helpers ──────────────────────────────────────────────────────────
  void load_params();
  void configure_gpio();
  void control_loop();
  void drive_motor(int in1, int in2, int pwm_pin, double pwm_value);

  // ── PID State ──────────────────────────────────────────────────────────────
  struct PID {
    double kp         = 30.0;
    double ki         = 10.0;
    double kd         =  0.0;
    double integral   =  0.0;
    double prev_error =  0.0;
    double min_output = -255.0;
    double max_output =  255.0;
    double offset_fwd =  0.0;
    double offset_bwd =  0.0;
  };

  PID left_pid_;
  PID right_pid_;
  std::mutex pid_mutex_;

  // ── Command state ──────────────────────────────────────────────────────────
  double left_cmd_  = 0.0;  // rad/s
  double right_cmd_ = 0.0;  // rad/s
  std::mutex cmd_mutex_;

  // ── Encoder / position tracking ────────────────────────────────────────────
  double left_pos_  = 0.0;  // accumulated radians
  double right_pos_ = 0.0;

  // ── Robot parameters ───────────────────────────────────────────────────────
  double pulses_per_rotation_ = 1400.0;
  double wheel_radius_        = 0.022;
  double wheel_separation_    = 0.0829;

  // ── pigpio ─────────────────────────────────────────────────────────────────
  int pigpio_handle_ = -1;
  int retry_counter_ = 0;

  // Encoder direction signs (+1 normal, -1 reversed).
  // Set via left_encoder_reversed / right_encoder_reversed params.
  // sign=1  →  left_ticks_ -= QEM  (flips left encoder for correct forward=positive)
  //            right_ticks_ += QEM (right encoder already correct polarity)
  // sign=-1 →  formulas revert to original (both same-sign → d_theta=0 when turning)
  int left_enc_sign_  = 1;   // corrected: use 1, not -1
  int right_enc_sign_ = 1;   // corrected: use 1, not -1

  // ── Timing ─────────────────────────────────────────────────────────────────
  rclcpp::Time prev_time_{0, 0, RCL_ROS_TIME};

  // ── ROS interfaces ─────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;

  // ── Parameter callback handle ──────────────────────────────────────────────
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace diffbot_gpio