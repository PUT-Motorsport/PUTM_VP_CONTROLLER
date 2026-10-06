#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/bms_hv_main.hpp"
#include "putm_vcl_interfaces/msg/setpoints.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values1.hpp"
#include "putm_vcl_interfaces/msg/steering_wheel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "putm_vcl_interfaces/msg/yaw_ref.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <Eigen/Dense>

// #include "putm_vcl_interfaces/msg/xsens_acceleration.hpp"
// #include "putm_vcl_interfaces/msg/xsens_rate_of_turn.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"

constexpr double MAX_MOMENT = 100;
constexpr double CAP_MOMENT = 100;
constexpr double Ku = 1.0/270.0;
constexpr bool enable_tc = true;

extern "C" {
#include "acados_solver_tv_nmpc.h"
#include "acados_c/ocp_nlp_interface.h"
}

using namespace std::chrono_literals;
using namespace putm_vcl_interfaces::msg;

using std::placeholders::_1;

class Controller : public rclcpp::Node {
 public:
  Controller();
  ~Controller();

 private:
  FrontboxDriverInput frontbox_driver_input;
  SteeringWheel steering_wheel;


  rclcpp::Publisher<Setpoints>::SharedPtr setpoints_publisher;
  rclcpp::Publisher<YawRef>::SharedPtr yaw_rate_ref_publisher;
  rclcpp::Subscription<FrontboxDriverInput>::SharedPtr frontbox_driver_input_subscriber;
  rclcpp::Subscription<SteeringWheel>::SharedPtr steering_wheel_subscriber;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_front_left_actual_values1_subscriber;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_front_right_actual_values1_subscriber;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_rear_left_actual_values1_subscriber;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_rear_right_actual_values1_subscriber;

  // rclcpp::Subscription<XsensAcceleration>::SharedPtr xsens_acceleration_ay_subscriber;
  // rclcpp::Subscription<XsensAcceleration>::SharedPtr xsens_acceleration_ax_subscriber;
  // rclcpp::Subscription<XsensRateOfTurn>::SharedPtr xsens_rate_of_turn_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr xsens_acceleration_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr xsens_angular_velocity_subscriber;

  rclcpp::Subscription<vectornav_msgs::msg::ImuGroup>::SharedPtr vn300_rate_of_turn_subscriber;
  rclcpp::Subscription<BmsHvMain>::SharedPtr bms_hv_main_subscriber;
  rclcpp::TimerBase::SharedPtr control_loop_timer;

  inline double convert_pedal_position(int16_t pedal_position);
  inline int32_t convert_torque(double torque);
  inline double convert_wheel_speed(double rpm);
  inline void convert_steering_angle(double steering_wheel_deg, double &delta_l_rad, double &delta_r_rad);
  inline void calculate_load_transfer(double ax, double ay, double &fz_fl, double &fz_fr, double &fz_rl, double &fz_rr);
  inline double referenceYawRate(double vx, double delta_deg);
  
  // Stany
  bool is_initialized;
  double speed_fl, speed_fr, speed_rl, speed_rr;
  double ay, ax, yaw_rate, batt_curr;
  Setpoints setpoints;
  YawRef yaw_ref;

  // Filtr
  double ax_raw_prev1 = 0.0, ax_raw_prev2 = 0.0;
  double ax_filt_prev1 = 0.0, ax_filt_prev2 = 0.0;

  double ay_raw_prev1 = 0.0, ay_raw_prev2 = 0.0;
  double ay_filt_prev1 = 0.0, ay_filt_prev2 = 0.0;

  double yaw_rate_raw_prev1 = 0.0, yaw_rate_raw_prev2 = 0.0;
  double yaw_rate_filt_prev1 = 0.0, yaw_rate_filt_prev2 = 0.0;

  const double b0 = 0.00988418;
  const double b1 = 0.01976837;
  const double b2 = 0.00988418;
  const double a1 = -1.69972730;
  const double a2 = 0.73926403;

  // MPC State Machine
  enum class ControlMode { MANUAL, WARMUP, BLEND_IN, NMPC, BLEND_OUT };
  ControlMode control_mode = ControlMode::MANUAL;
  double blend_alpha = 0.0;

  static constexpr double SPEED_WARMUP     = 2.0;
  static constexpr double SPEED_ENTER_NMPC = 3.0;
  static constexpr double SPEED_EXIT_NMPC  = 2.0;
  static constexpr double BLEND_DURATION_S = 1.0;

  // Wskaźniki i bufory ACADOS
  tv_nmpc_solver_capsule *acados_capsule;
  ocp_nlp_config *nlp_config;
  ocp_nlp_dims *nlp_dims;
  ocp_nlp_in *nlp_in;
  ocp_nlp_out *nlp_out;

  double lbx0[TV_NMPC_NBX0];
  double ubx0[TV_NMPC_NBX0];
  double p_val[TV_NMPC_NP];
  double x_k1[TV_NMPC_NX];

  const double dt = 0.01;

  // TC
  double tau_final[4];
  double prev_tau_nmpc[4];

  // Parametry TC
  const double R_e = 0.193;

  const double kappa_limit = 1.05;

  // EKF
  Eigen::Vector2d ekf_x;
  Eigen::Matrix2d ekf_P;
  Eigen::Matrix2d ekf_Q;
  Eigen::Matrix2d ekf_I;

  inline void estimate_velocity_ekf(double ax, double ay, double r, double w_fl, double w_fr, double w_rl, double w_rr, double delta_l, double delta_r, double &vx_est, double &vy_est);

  inline const char* controlModeToString(ControlMode mode) {
  switch (mode) {
    case ControlMode::MANUAL:    return "MANUAL";
    case ControlMode::WARMUP:    return "WARMUP";
    case ControlMode::BLEND_IN:  return "BLEND_IN";
    case ControlMode::NMPC:      return "NMPC";
    case ControlMode::BLEND_OUT: return "BLEND_OUT";
    default:                     return "UNKNOWN";
  }
}

  void frontbox_driver_input_topic_callback(const FrontboxDriverInput msg);
  void steering_wheel_callback(const SteeringWheel::SharedPtr msg);
  void amk_actual_values1_callback(const AmkActualValues1 msg);
  void amk_actual_values2_callback(const AmkActualValues1 msg);
  void amk_actual_values3_callback(const AmkActualValues1 msg);
  void amk_actual_values4_callback(const AmkActualValues1 msg);
  
  // void xsens_acceleration_ay_callback(const XsensAcceleration msg);
  // void xsens_acceleration_ax_callback(const XsensAcceleration msg);
  // void xsens_rate_of_turn_callback(const XsensRateOfTurn msg);
  void xsens_acceleration_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void xsens_angular_velocity_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  void vn300_rate_of_turn_callback(const vectornav_msgs::msg::ImuGroup msg);
  void bms_hv_main_callback(const BmsHvMain msg);

  void control_loop();
};

Controller::Controller()
    : Node("controller"),
      setpoints_publisher(this->create_publisher<Setpoints>("putm_vcl/setpoints", 1)),
      yaw_rate_ref_publisher(this->create_publisher<YawRef>("yaw_ref", 1)),
      frontbox_driver_input_subscriber(this->create_subscription<FrontboxDriverInput>("putm_vcl/frontbox_driver_input", 1, std::bind(&Controller::frontbox_driver_input_topic_callback, this, _1))),
      steering_wheel_subscriber(this->create_subscription<SteeringWheel>("putm_vcl/steering_wheel", 1, std::bind(&Controller::steering_wheel_callback, this, _1))),
      amk_front_left_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/front/left/actual_values1", 1, std::bind(&Controller::amk_actual_values1_callback, this, _1))),
      amk_front_right_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/front/right/actual_values1", 1, std::bind(&Controller::amk_actual_values2_callback, this, _1))),
      amk_rear_left_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/rear/left/actual_values1", 1, std::bind(&Controller::amk_actual_values3_callback, this, _1))),
      amk_rear_right_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/rear/right/actual_values1", 1, std::bind(&Controller::amk_actual_values4_callback, this, _1))),
      // xsens_acceleration_ay_subscriber(this->create_subscription<XsensAcceleration>("putm_vcl/xsens_acceleration", 1, std::bind(&Controller::xsens_acceleration_ay_callback, this, _1))),
      // xsens_acceleration_ax_subscriber(this->create_subscription<XsensAcceleration>("putm_vcl/xsens_acceleration", 1, std::bind(&Controller::xsens_acceleration_ax_callback, this, _1))),
      // xsens_rate_of_turn_subscriber(this->create_subscription<XsensRateOfTurn>("putm_vcl/xsens_rate_of_turn", 1, std::bind(&Controller::xsens_rate_of_turn_callback, this, _1))),
      xsens_acceleration_subscriber(this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/imu/acceleration", 1, std::bind(&Controller::xsens_acceleration_callback, this, _1))),
      xsens_angular_velocity_subscriber(this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/imu/angular_velocity", 1, std::bind(&Controller::xsens_angular_velocity_callback, this, _1))),      
      vn300_rate_of_turn_subscriber(this->create_subscription<vectornav_msgs::msg::ImuGroup>("vectornav/raw/imu", 1,  std::bind(&Controller::vn300_rate_of_turn_callback, this, _1))),
      bms_hv_main_subscriber(this->create_subscription<BmsHvMain>("putm_vcl/bms_hv_main", 1,  std::bind(&Controller::bms_hv_main_callback, this, _1))),
      control_loop_timer(this->create_wall_timer(10ms, std::bind(&Controller::control_loop, this))),
      is_initialized(false),
      speed_fl(0), speed_fr(0), speed_rl(0), speed_rr(0),
      ay(0.0), ax(0.0), yaw_rate(0.0), batt_curr(0.0)
      {

      acados_capsule = tv_nmpc_acados_create_capsule();
      int status = tv_nmpc_acados_create(acados_capsule);
      if (status) {
        RCLCPP_FATAL(this->get_logger(), "Acados solver init failed with status: %d", status);
      }

      nlp_config = tv_nmpc_acados_get_nlp_config(acados_capsule);
      nlp_dims   = tv_nmpc_acados_get_nlp_dims(acados_capsule);
      nlp_in     = tv_nmpc_acados_get_nlp_in(acados_capsule);
      nlp_out    = tv_nmpc_acados_get_nlp_out(acados_capsule);

      for (int i = 0; i < 4; i++) {
        tau_final[i] = 0.0;
        prev_tau_nmpc[i] = 0.0;
      }

      ekf_x << 0.0, 0.0;
      ekf_P = Eigen::Matrix2d::Identity() * 0.1;
      ekf_I = Eigen::Matrix2d::Identity();

      }

Controller::~Controller() {
  tv_nmpc_acados_free(acados_capsule);
  tv_nmpc_acados_free_capsule(acados_capsule);
}

void Controller::frontbox_driver_input_topic_callback(const FrontboxDriverInput msg) { frontbox_driver_input = msg; }
void Controller::steering_wheel_callback(const SteeringWheel::SharedPtr msg) { steering_wheel = *msg; }
void Controller::amk_actual_values1_callback(const AmkActualValues1 msg) { speed_fl = abs(msg.actual_velocity); }
void Controller::amk_actual_values2_callback(const AmkActualValues1 msg) { speed_fr = abs(msg.actual_velocity); }
void Controller::amk_actual_values3_callback(const AmkActualValues1 msg) { speed_rl = abs(msg.actual_velocity); }
void Controller::amk_actual_values4_callback(const AmkActualValues1 msg) { speed_rr = abs(msg.actual_velocity); }

// void Controller::xsens_acceleration_ay_callback(const XsensAcceleration msg) { (void)msg; /* ay = msg.acc_y; */ }
// void Controller::xsens_acceleration_ax_callback(const XsensAcceleration msg) { (void)msg; /* ax = msg.acc_x; */ }
// void Controller::xsens_rate_of_turn_callback(const XsensRateOfTurn msg) { (void)msg; /* yaw_rate = msg.gyr_z; */ }

void Controller::xsens_acceleration_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
  // double ax_raw = msg->vector.y;
  // double ay_raw = -msg->vector.x;

  // ax = b0*ax_raw + b1*ax_raw_prev1 + b2*ax_raw_prev2 - a1*ax_filt_prev1 - a2*ax_filt_prev2;

  // ay = b0*ay_raw + b1*ay_raw_prev1 + b2*ay_raw_prev2 - a1*ay_filt_prev1 - a2*ay_filt_prev2;

  // ax_raw_prev2 = ax_raw_prev1;
  // ax_raw_prev1 = ax_raw;
  // ax_filt_prev2 = ax_filt_prev1;
  // ax_filt_prev1 = ax;

  // ay_raw_prev2 = ay_raw_prev1;
  // ay_raw_prev1 = ay_raw;
  // ay_filt_prev2 = ay_filt_prev1;
  // ay_filt_prev1 = ay;
}

void Controller::xsens_angular_velocity_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {

  // double yaw_rate_raw = msg->vector.z;

  // yaw_rate = b0*yaw_rate_raw + b1*yaw_rate_raw_prev1 + b2*yaw_rate_raw_prev2 - a1*yaw_rate_filt_prev1 - a2*yaw_rate_filt_prev2;

  // yaw_rate_raw_prev2 = yaw_rate_raw_prev1;
  // yaw_rate_raw_prev1 = yaw_rate_raw;
  // yaw_rate_filt_prev2 = yaw_rate_filt_prev1;
  // yaw_rate_filt_prev1 = yaw_rate;
}

void Controller::vn300_rate_of_turn_callback(const vectornav_msgs::msg::ImuGroup msg) {
  double ax_raw = msg.accel.x * -1;
  double ay_raw = msg.accel.y * -1;
  double yaw_rate_raw = msg.angularrate.z;

  yaw_rate = b0*yaw_rate_raw + b1*yaw_rate_raw_prev1 + b2*yaw_rate_raw_prev2 - a1*yaw_rate_filt_prev1 - a2*yaw_rate_filt_prev2;

  yaw_rate_raw_prev2 = yaw_rate_raw_prev1;
  yaw_rate_raw_prev1 = yaw_rate_raw;
  yaw_rate_filt_prev2 = yaw_rate_filt_prev1;
  yaw_rate_filt_prev1 = yaw_rate;

  ax = b0*ax_raw + b1*ax_raw_prev1 + b2*ax_raw_prev2 - a1*ax_filt_prev1 - a2*ax_filt_prev2;

  ay = b0*ay_raw + b1*ay_raw_prev1 + b2*ay_raw_prev2 - a1*ay_filt_prev1 - a2*ay_filt_prev2;

  ax_raw_prev2 = ax_raw_prev1;
  ax_raw_prev1 = ax_raw;
  ax_filt_prev2 = ax_filt_prev1;
  ax_filt_prev1 = ax;

  ay_raw_prev2 = ay_raw_prev1;
  ay_raw_prev1 = ay_raw;
  ay_filt_prev2 = ay_filt_prev1;
  ay_filt_prev1 = ay;
}

void Controller::bms_hv_main_callback(const BmsHvMain msg) { batt_curr = msg.current; }



void Controller::control_loop() {
  auto start_time = std::chrono::high_resolution_clock::now();

  double pedal = convert_pedal_position(frontbox_driver_input.pedal_position);
  double steering_angle_deg = ((double)steering_wheel.steering_wheel_position /135 * 50);

  double w_fl = convert_wheel_speed(speed_fl);
  double w_fr = convert_wheel_speed(speed_fr);
  double w_rl = convert_wheel_speed(speed_rl);
  double w_rr = convert_wheel_speed(speed_rr);

  double delta_l_rad = 0.0;
  double delta_r_rad = 0.0;
  convert_steering_angle(steering_angle_deg, delta_l_rad, delta_r_rad);
  
  double vx_est = 1.0;
  double vy_est = 0.0;
  estimate_velocity_ekf(ax, ay, yaw_rate, w_fl, w_fr, w_rl, w_rr, delta_l_rad, delta_r_rad, vx_est, vy_est);
  double delta_avg_rad = (delta_l_rad + delta_r_rad) / 2.0;
  double yaw_rate_ref = referenceYawRate(vx_est, delta_avg_rad * 180.0 / M_PI);
  double fz_fl = 0.0, fz_fr = 0.0, fz_rl = 0.0, fz_rr = 0.0;
  calculate_load_transfer(ax, ay, fz_fl, fz_fr, fz_rl, fz_rr);
  double manual_torque = pedal * MAX_MOMENT;
  ControlMode prev_mode = control_mode;

  switch (control_mode) {
    case ControlMode::MANUAL:
      if (vx_est >= SPEED_WARMUP) control_mode = ControlMode::WARMUP;
      break;
    case ControlMode::WARMUP:
      if (vx_est < SPEED_WARMUP) { control_mode = ControlMode::MANUAL; is_initialized = false; }
      else if (vx_est >= SPEED_ENTER_NMPC) { control_mode = ControlMode::BLEND_IN; }
      break;
    case ControlMode::BLEND_IN:
      blend_alpha += dt / BLEND_DURATION_S;
      if (blend_alpha >= 1.0) { blend_alpha = 1.0; control_mode = ControlMode::NMPC; }
      if (vx_est < SPEED_EXIT_NMPC) control_mode = ControlMode::BLEND_OUT;
      break;
    case ControlMode::NMPC:
      if (vx_est < SPEED_EXIT_NMPC) control_mode = ControlMode::BLEND_OUT;
      break;
    case ControlMode::BLEND_OUT:
      blend_alpha -= dt / BLEND_DURATION_S;
      if (blend_alpha <= 0.0) { blend_alpha = 0.0; control_mode = ControlMode::WARMUP; }
      if (vx_est >= SPEED_ENTER_NMPC) control_mode = ControlMode::BLEND_IN;
      break;
  }

  if (control_mode != prev_mode) {
    RCLCPP_INFO(this->get_logger(),
        "State machine: %s -> %s | vx_est=%.2f blend_alpha=%.2f",
        controlModeToString(prev_mode), controlModeToString(control_mode),
        vx_est, blend_alpha);
  }

  bool solver_active = (control_mode != ControlMode::MANUAL);

  // Low speed mode z manualnym sterowaniem momentem
  if (!solver_active) {

    tau_final[0] = manual_torque;
    tau_final[1] = manual_torque;
    tau_final[2] = manual_torque;
    tau_final[3] = manual_torque;

    prev_tau_nmpc[0] = manual_torque;
    prev_tau_nmpc[1] = manual_torque;
    prev_tau_nmpc[2] = manual_torque;
    prev_tau_nmpc[3] = manual_torque;

    is_initialized = false;
  }
  else {

    double t_ref = manual_torque * 4; 

    p_val[0] = yaw_rate_ref; 
    p_val[1] = delta_l_rad;
    p_val[2] = delta_r_rad;
    p_val[3] = fz_fl;
    p_val[4] = fz_fr;
    p_val[5] = fz_rl;
    p_val[6] = fz_rr;
    p_val[7] = t_ref;

    for (int i = 0; i <= TV_NMPC_N; i++) {
      tv_nmpc_acados_update_params(acados_capsule, i, p_val, 8);
    }

    // Przygotowanie bufora stanu x0
    lbx0[0] = vx_est; ubx0[0] = vx_est;
    lbx0[1] = vy_est; ubx0[1] = vy_est;
    lbx0[2] = yaw_rate; ubx0[2] = yaw_rate;
    lbx0[3] = w_fl; ubx0[3] = w_fl;
    lbx0[4] = w_fr; ubx0[4] = w_fr;
    lbx0[5] = w_rl; ubx0[5] = w_rl;
    lbx0[6] = w_rr; ubx0[6] = w_rr;
    
    // Feedback stanów wewnętrznych
    lbx0[7] = tau_final[0]; ubx0[7] = tau_final[0];
    lbx0[8] = tau_final[1]; ubx0[8] = tau_final[1];
    lbx0[9] = tau_final[2]; ubx0[9] = tau_final[2];
    lbx0[10]= tau_final[3]; ubx0[10]= tau_final[3];

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", ubx0);

    // COLD START 
    if (!is_initialized) {
      double x_init[TV_NMPC_NX] = {vx_est, vy_est, yaw_rate, w_fl, w_fr, w_rl, w_rr, 
                                   tau_final[0], tau_final[1], tau_final[2], tau_final[3]};
      double u_init[TV_NMPC_NU] = {0.0, 0.0, 0.0, 0.0};

      for (int i = 0; i < TV_NMPC_N; i++) {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_init);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u_init);
      }
      ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, TV_NMPC_N, "x", x_init);
      is_initialized = true;
    }
    
    // SOLVE
    int status = tv_nmpc_acados_solve(acados_capsule);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_ms = end_time - start_time;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
        "Czas NMPC: %.3f ms | Status: %d", elapsed_ms.count(), status);
    
    if (status != 0) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500, "NMPC Fail. Przytrzymanie momentu.");
    is_initialized = false;
    }
    else {

      // Pobranie zoptymalizowanych momentów
      ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", x_k1);
      prev_tau_nmpc[0] = x_k1[7]; 
      prev_tau_nmpc[1] = x_k1[8]; 
      prev_tau_nmpc[2] = x_k1[9]; 
      prev_tau_nmpc[3] = x_k1[10];

      // TC
      double w_actual[4] = {w_fl, w_fr, w_rl, w_rr};
      
      for (int i = 0; i < 4; i++) {
          double blended = (1.0 - blend_alpha) * manual_torque + blend_alpha * prev_tau_nmpc[i];

          double max_safe_v_wheel = (vx_est * kappa_limit);
          if (enable_tc && pedal > 0.05) {
              double current_v_wheel = w_actual[i] * R_e;
              double dynamic_max_torque = MAX_MOMENT;
              if (current_v_wheel > max_safe_v_wheel) {
                  double speed_excess = current_v_wheel - max_safe_v_wheel;
                  dynamic_max_torque = std::max(0.0, MAX_MOMENT - speed_excess * 80.0);
              }
              tau_final[i] = std::clamp(blended, -10.0, dynamic_max_torque);
          } else {
              tau_final[i] = std::clamp(blended, -10.0, MAX_MOMENT);
          }
      }
    }
  }
  
  if (pedal < 0.01) {
    tau_final[0] = 0.0;
    tau_final[1] = 0.0;
    tau_final[2] = 0.0;
    tau_final[3] = 0.0;
  }

  // Publikacja 
  yaw_ref.yaw_rate_ref = yaw_rate_ref; 
  yaw_ref.vx_est = vx_est;
  yaw_ref.vy_est = vy_est;
  yaw_ref.filtered_ax = ax;
  yaw_ref.filtered_ay = ay;
  yaw_ref.fz_fl = fz_fl;
  yaw_ref.fz_fr = fz_fr;
  yaw_ref.fz_rl = fz_rl;
  yaw_ref.fz_rr = fz_rr;
  yaw_rate_ref_publisher->publish(yaw_ref);

  setpoints.front_left.torque = convert_torque(-tau_final[0]);
  setpoints.front_right.torque = convert_torque(tau_final[1]);
  setpoints.rear_left.torque = convert_torque(tau_final[2]);
  setpoints.rear_right.torque = convert_torque(tau_final[3]);
  setpoints_publisher->publish(setpoints);
}

inline double Controller::convert_pedal_position(int16_t pedal_position) {
  static constexpr double PEDAL_SCALER = 450.0;
  return (((double)pedal_position) / PEDAL_SCALER);
}


inline double Controller::convert_wheel_speed(double rpm) {
  
  double gear_ratio = 11; 
  return (rpm * (M_PI / 30.0)) / gear_ratio;
}

inline double Controller::referenceYawRate(double vx, double delta_deg)
{
    const double delta_wheel = (delta_deg * M_PI / 180.0);
    const double denominator = 1.53 * (0.1 + Ku * vx * vx);

    if (std::fabs(denominator) < 1e-6)
        return 0.0;

    const double yaw_rate = vx * delta_wheel / denominator;

    return std::clamp(yaw_rate, -4.0, 4.0);
}

inline void Controller::convert_steering_angle(double steering_wheel_deg, double &delta_l_rad, double &delta_r_rad) {
  
  double S_abs = std::abs(steering_wheel_deg);
  
  double inner_wheel = 0.000410 * S_abs * S_abs + 0.2554 * S_abs;
  double outer_wheel = -0.0000942 * S_abs * S_abs + 0.2543 * S_abs;

  if (steering_wheel_deg >= 0.0) {
    delta_l_rad = inner_wheel;
    delta_r_rad = outer_wheel;
  } else {
    delta_l_rad = -outer_wheel;
    delta_r_rad = -inner_wheel;
  }

  // Konwersja na radiany
  delta_l_rad *= (M_PI / 180.0);
  delta_r_rad *= (M_PI / 180.0);
}

inline void Controller::calculate_load_transfer(double ax_sensor, double ay_sensor, double &fz_fl, double &fz_fr, double &fz_rl, double &fz_rr) {

  const double m = 300.0;     // Masa beniga
  const double g = 9.81;      // przyspieszenie
  const double a = 0.765;     // odleglosc cg od przedniej osi
  const double b = 0.765;     // odleglosc cg od tylnej osi 
  const double c = 0.621;     // polowa rostawu kol
  const double L = a + b;     // rozstaw osi
  const double h = 0.3;     // wysokosc cg

  double Fz_static_front = (m * g * b) / (2.0 * L);
  double Fz_static_rear  = (m * g * a) / (2.0 * L);

  double dFz_long = (m * h * ax_sensor) / (2.0 * L);
  double dFz_lat_front = (m * h * ay_sensor * b) / (2.0 * L * c);
  double dFz_lat_rear  = (m * h * ay_sensor * a) / (2.0 * L * c);

  fz_fl = Fz_static_front - dFz_long - dFz_lat_front; 
  fz_fr = Fz_static_front - dFz_long + dFz_lat_front; 
  
  fz_rl = Fz_static_rear  + dFz_long - dFz_lat_rear;
  fz_rr = Fz_static_rear  + dFz_long + dFz_lat_rear;

  if (fz_fl < 10.0) fz_fl = 10.0;
  if (fz_fr < 10.0) fz_fr = 10.0;
  if (fz_rl < 10.0) fz_rl = 10.0;
  if (fz_rr < 10.0) fz_rr = 10.0;
}

inline int32_t Controller::convert_torque(double torque) {
  static constexpr double TORQUE_SCALER = 1000.0;
  return (int32_t)((torque / CAP_MOMENT) * TORQUE_SCALER);
}

inline void Controller::estimate_velocity_ekf(double ax, double ay, double r, double w_fl, double w_fr, double w_rl, double w_rr, double delta_l, double delta_r, double &vx_est, double &vy_est) {

    const double c   = 0.621;
    const double b   = 0.765;

    const double q_vx      = 5.50;
    const double q_vy      = 0.00010;
    const double r_wheels  = 0.005777;
    const double r_yaw     = 0.007262;
    const double r_zlvu    = 0.00969129;
    const double ax_pen    = 0.006621;

    Eigen::Matrix2d Q;
    Q << q_vx, 0.0,
         0.0,  q_vy;

    double delta = (delta_l + delta_r) / 2.0;
    double ax_safe = std::clamp(ax, -18.0, 18.0);
    double ay_safe = std::clamp(ay, -20.0, 20.0);

    // PREDYKCJA
    Eigen::Vector2d x_pred;
    x_pred(0) = ekf_x(0) + (ax_safe + ekf_x(1) * r) * dt;
    x_pred(1) = ekf_x(1) + (ay_safe - ekf_x(0) * r) * dt;

    Eigen::Matrix2d F;
    F << 1.0,      r * dt,
        -r * dt,   1.0;

    Eigen::Matrix2d P_pred = F * ekf_P * F.transpose() + Q;

    // POMIARY KÓŁ
    Eigen::Vector4d z_wheels, z_pred_wheels;
    z_wheels(0) = w_fl * R_e * std::cos(delta_l);
    z_wheels(1) = w_fr * R_e * std::cos(delta_r);
    z_wheels(2) = w_rl * R_e;
    z_wheels(3) = w_rr * R_e;

    z_pred_wheels(0) = x_pred(0) - r * c;
    z_pred_wheels(1) = x_pred(0) + r * c;
    z_pred_wheels(2) = x_pred(0) - r * c;
    z_pred_wheels(3) = x_pred(0) + r * c;

    Eigen::Vector4d y_wheels = z_wheels - z_pred_wheels;

    // DETEKCJA TRYBU
    double avg_wheel_speed = z_wheels.cwiseAbs().mean();
    bool is_stopped  = (avg_wheel_speed < 0.1) && (x_pred(0) < 0.3);
    bool is_straight = (std::abs(delta) < 0.03) && (std::abs(r) < 0.05);

    Eigen::Matrix<double, 5, 1> z_full, z_pred_full;
    z_full.head<4>()      = z_wheels;
    z_pred_full.head<4>() = z_pred_wheels;
    z_pred_full(4)        = x_pred(1);

    Eigen::Matrix<double, 5, 2> H = Eigen::Matrix<double, 5, 2>::Zero();
    H.col(0) << 1, 1, 1, 1, 0;
    H.col(1) << 0, 0, 0, 0, 1;

    Eigen::Matrix<double, 5, 5> R_adaptive = Eigen::Matrix<double, 5, 5>::Zero();

    if (is_stopped) {
        z_full(4)   = 0.0;
        R_adaptive  = Eigen::Matrix<double, 5, 5>::Identity() * 1e-4;
    } else {
        int wheels_slipping = 0;
        
        for (int i = 0; i < 4; i++) {
            double slip = 0.0;
            if (x_pred(0) > 1.0) {
                slip = std::abs(y_wheels(i)) / x_pred(0);
            }
            
            if (slip > 0.4) {
                wheels_slipping++;
                R_adaptive(i, i) = 5.0; 
            } else {
                R_adaptive(i, i) = r_wheels + std::abs(ax_safe) * ax_pen;
            }
        }

        if (wheels_slipping >= 3) {
            for (int i = 0; i < 4; i++) {
                R_adaptive(i, i) = 1.0;
            }
        }

        // Pomiar boczny
        if (is_straight) {
            z_full(4)    = 0.0;
            R_adaptive(4, 4) = r_zlvu;
        } else {
            z_full(4)    = r * b;
            R_adaptive(4, 4) = r_yaw + std::abs(ay_safe) * 0.2;
        }
    }

    // KOREKCJA
    Eigen::Matrix<double, 5, 1> y = z_full - z_pred_full;
    Eigen::Matrix<double, 5, 5> S = H * P_pred * H.transpose() + R_adaptive;
    Eigen::Matrix<double, 2, 5> K = P_pred * H.transpose() * S.inverse();

    ekf_x = x_pred + K * y;
    ekf_P = (ekf_I - K * H) * P_pred;

    // ZABEZPIECZENIA
    if (ekf_x(0) < 0.0) ekf_x(0) = 0.0;

    vx_est = ekf_x(0);
    vy_est = std::tanh(ekf_x(1) / 2.0);
}



int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
}
