#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/bms_hv_main.hpp"
#include "putm_vcl_interfaces/msg/setpoints.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values1.hpp"
#include "putm_vcl_interfaces/msg/steering_wheel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "putm_vcl_interfaces/msg/yaw_ref.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>


// POCZĄTEK INICJALIZACJI STAŁYCH POD REGULATOR PI/PID!!!!!!!!!!!

 // Limity pod regulator na furę na kobyłkach (uniesionej w górzę)
constexpr double TAU_MIN = -20.0;
constexpr double TAU_MAX = 200.0;

//Limity pod regulator na furę jeżdżącą
// constexpr double TAU_MIN_real = 0.0;
// constexpr double TAU_MAX_real = 500.0;

//parametry kierownicy
// constexpr double Kierwonica_ratio = 0.3;

constexpr int SWITCH_KEY_MOTION_MODE_DEFAULT = 3;

constexpr double GAIN_CONST_DEFAULT = 1.0;

// constexpr double KPRO_FR_DEFAULT = 87.14;
// constexpr double KPRO_FL_DEFAULT = 102.53;
// constexpr double KPRO_RR_DEFAULT = 70.0;
// constexpr double KPRO_RL_DEFAULT = 80.26;

// constexpr double KINT_FR_DEFAULT = 270.79;
// constexpr double KINT_FL_DEFAULT = 139.43;
// constexpr double KINT_RR_DEFAULT = 215.35;
// constexpr double KINT_RL_DEFAULT = 240.01;


constexpr double KPRO_FR_DEFAULT = 73.71;
constexpr double KPRO_FL_DEFAULT = 79.27;
constexpr double KPRO_RR_DEFAULT = 76.24;
constexpr double KPRO_RL_DEFAULT = 147.87;

constexpr double KINT_FR_DEFAULT = 88.85;
constexpr double KINT_FL_DEFAULT = 87.41;
constexpr double KINT_RR_DEFAULT = 99.26;
constexpr double KINT_RL_DEFAULT = 293.40;

// Parametry fizyczne bolidu:
constexpr double WHEEL_RADIUS = 0.198;
constexpr double GEAR_RATIO = 11.0;
constexpr double RPM_TO_MPS = 2.0 * M_PI * WHEEL_RADIUS / (60.0 * GEAR_RATIO);

// Ograniczenia
constexpr double MAX_DTAU = 40.0; // Ograniczenie w zmianie Torque


// START ZMIENNE GLOBALNE
// Zmienne globalne - GLOBAL (zmienne dotyczące wszystkich modów jazdy)
  double tau_prev[4] = {0.0, 0.0, 0.0, 0.0};
  double tau_final[4] = {0.0, 0.0, 0.0, 0.0};

  double velocity_set_front_left = 0.0;
  double velocity_set_front_right = 0.0;
  double velocity_set_rear_left = 0.0;
  double velocity_set_rear_right = 0.0;





// Zmienne globalne - MONO MODE (CASE 2)
constexpr double VELOCITY_SET_MONO_MODE = 5.0;

// Zmienne globalne VECTORING MODE (CASE 3)
constexpr double VECTORING_DIVIDER = 120.0;


constexpr bool ODPOWIEDZ_SKOKOWA = false;
constexpr double TAU_SKOKOWA[4] = {80.0, 80.0, 50.0, 50.0};



 // KONIEC ZMIENNYCH GLOBALNYCH

// KONIEC INICJALIZACJI STAŁYCH POD REGULATOR PI/PID!!!!!!!!!!!

constexpr double refresh_rate = 0.01; // refresh rate of controller.cpp in [ms]


// #include "putm_vcl_interfaces/msg/xsens_acceleration.hpp"
// #include "putm_vcl_interfaces/msg/xsens_rate_of_turn.hpp"
// #include "vectornav_msgs/msg/imu_group.hpp"

constexpr double MAX_MOMENT = 4 * 9.8 * 11;
constexpr double CAP_MOMENT = 143;
constexpr double Ku = 1.0/50.0;
constexpr bool enable_tc = true;

// extern "C" {
// #include "acados_solver_tv_nmpc.h"
// #include "acados_c/ocp_nlp_interface.h"
// }

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

   // SWITCH KEY dla regulatora PI
  uint16_t switch_key_motion_modes_ = 1;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  // KONIEC INICJALIZACJI

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

  // rclcpp::Subscription<vectornav_msgs::msg::ImuGroup>::SharedPtr vn300_rate_of_turn_subscriber;
  rclcpp::Subscription<BmsHvMain>::SharedPtr bms_hv_main_subscriber;
  rclcpp::TimerBase::SharedPtr control_loop_timer;

  inline double convert_pedal_position(int16_t pedal_position);
  inline int32_t convert_torque(double torque);
  inline double convert_wheel_speed(double rpm);
  inline void convert_steering_angle(double steering_wheel_deg, double &delta_l_rad, double &delta_r_rad);
  inline void calculate_load_transfer(double ax, double ay, double &fz_fl, double &fz_fr, double &fz_rl, double &fz_rr);
  inline double referenceYawRate(double vx, double delta_deg);
  
  double integral_front_left = 0.0, integral_front_right = 0.0;
  double integral_rear_left = 0.0, integral_rear_right = 0.0;

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

  // Wskaźniki i bufory ACADOS
  // tv_nmpc_solver_capsule *acados_capsule;
  // ocp_nlp_config *nlp_config;
  // ocp_nlp_dims *nlp_dims;
  // ocp_nlp_in *nlp_in;
  // ocp_nlp_out *nlp_out;

  // double lbx0[TV_NMPC_NBX0];
  // double ubx0[TV_NMPC_NBX0];
  // double p_val[TV_NMPC_NP];
  // double x_k1[TV_NMPC_NX];

  const double dt = 0.01;

  // TC
  // double tau_final[4];
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

  // void vn300_rate_of_turn_callback(const vectornav_msgs::msg::ImuGroup msg);
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
      // vn300_rate_of_turn_subscriber(this->create_subscription<vectornav_msgs::msg::ImuGroup>("vectornav/raw/imu", 1,  std::bind(&Controller::vn300_rate_of_turn_callback, this, _1))),
      bms_hv_main_subscriber(this->create_subscription<BmsHvMain>("putm_vcl/bms_hv_main", 1,  std::bind(&Controller::bms_hv_main_callback, this, _1))),
      control_loop_timer(this->create_wall_timer(10ms, std::bind(&Controller::control_loop, this))),
      is_initialized(false),
      speed_fl(0), speed_fr(0), speed_rl(0), speed_rr(0),
      ay(0.0), ax(0.0), yaw_rate(0.0), batt_curr(0.0)
      {
      // acados_capsule = tv_nmpc_acados_create_capsule();
      // int status = tv_nmpc_acados_create(acados_capsule);
      // if (status) {
      //   RCLCPP_FATAL(this->get_logger(), "Acados solver init failed with status: %d", status);
      // }
      
      // nlp_config = tv_nmpc_acados_get_nlp_config(acados_capsule);
      // nlp_dims   = tv_nmpc_acados_get_nlp_dims(acados_capsule);
      // nlp_in     = tv_nmpc_acados_get_nlp_in(acados_capsule);
      // nlp_out    = tv_nmpc_acados_get_nlp_out(acados_capsule);

      for (int i = 0; i < 4; i++) {
        // tau_final[i] = 0.0;
        prev_tau_nmpc[i] = 0.0;
      }

      ekf_x << 0.0, 0.0;
      ekf_P = Eigen::Matrix2d::Identity() * 0.1;
      ekf_I = Eigen::Matrix2d::Identity();



      // SWITCHING KEY INICJALIZACJA
      switch_key_motion_modes_ = declare_parameter<int>("switch_key_motion_modes", SWITCH_KEY_MOTION_MODE_DEFAULT);
      declare_parameter("gain_const", GAIN_CONST_DEFAULT);

      declare_parameter("Kpro_fr", KPRO_FR_DEFAULT);  declare_parameter("Kpro_fl", KPRO_FL_DEFAULT);
      declare_parameter("Kpro_rr", KPRO_RR_DEFAULT);  declare_parameter("Kpro_rl", KPRO_RL_DEFAULT);

      declare_parameter("Kint_fr", KINT_FR_DEFAULT);  declare_parameter("Kint_fl", KINT_FL_DEFAULT);
      declare_parameter("Kint_rr", KINT_RR_DEFAULT);  declare_parameter("Kint_rl", KINT_RL_DEFAULT);
            param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          rcl_interfaces::msg::SetParametersResult res;
          res.successful = true;

          for (const auto& p : params) {
            const std::string name = p.get_name();

            if (name == "switch_key_motion_modes") {
              int v = p.as_int();
              if (v < 0 || v > 3) {
                res.successful = false;
                res.reason = "switch_key_motion_modes musi byc w zakresie 0-3";
                RCLCPP_WARN(get_logger(), "Odrzucono %s = %d (zakres 0-3)", name.c_str(), v);
                return res;
              }
            } else if (name == "gain_const" || name.rfind("Kpro_", 0) == 0 || name.rfind("Kint_", 0) == 0) {
              double v = p.as_double();
              if (!std::isfinite(v) || v < 0.0) {
                res.successful = false;
                res.reason = name + " musi byc liczba >= 0";
                RCLCPP_WARN(get_logger(), "Odrzucono %s = %f (musi byc >= 0)", name.c_str(), v);
                return res;
              }
            }

            RCLCPP_INFO(get_logger(), "Zmieniono %s na %s",
                        name.c_str(), p.value_to_string().c_str());
          }
          return res;
        });
        // KONIEC INIZJACLIZACJI
      }

Controller::~Controller() {
  // tv_nmpc_acados_free(acados_capsule);
  // tv_nmpc_acados_free_capsule(acados_capsule);
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
  double ax_raw = msg->vector.y;
  double ay_raw = -msg->vector.x;

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

void Controller::xsens_angular_velocity_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {

  double yaw_rate_raw = msg->vector.z;

  yaw_rate = b0*yaw_rate_raw + b1*yaw_rate_raw_prev1 + b2*yaw_rate_raw_prev2 - a1*yaw_rate_filt_prev1 - a2*yaw_rate_filt_prev2;

  yaw_rate_raw_prev2 = yaw_rate_raw_prev1;
  yaw_rate_raw_prev1 = yaw_rate_raw;
  yaw_rate_filt_prev2 = yaw_rate_filt_prev1;
  yaw_rate_filt_prev1 = yaw_rate;
}

// void Controller::vn300_rate_of_turn_callback(const vectornav_msgs::msg::ImuGroup msg) {
//   // double ax_raw = msg.accel.x * -1;
//   // double ay_raw = msg.accel.y * -1;
//   // yaw_rate = msg.angularrate.z;

//   // ax_filtered = lp_alpha_acc * ax_raw  + (1.0 - lp_alpha_acc) * ax_filtered;
//   // ay_filtered = lp_alpha_acc * ay_raw  + (1.0 - lp_alpha_acc) * ay_filtered;

//   // ax = ax_filtered;
//   // ay = ay_filtered;
// }

void Controller::bms_hv_main_callback(const BmsHvMain msg) { batt_curr = msg.current; }



void Controller::control_loop() {
  auto start_time = std::chrono::high_resolution_clock::now();

  double pedal = convert_pedal_position(frontbox_driver_input.pedal_position);
  double steering_angle_deg = steering_wheel.steering_wheel_position * -1;

  double w_fl = convert_wheel_speed(speed_fl);
  double w_fr = convert_wheel_speed(speed_fr);
  double w_rl = convert_wheel_speed(speed_rl);
  double w_rr = convert_wheel_speed(speed_rr);

  double delta_l_rad = 0.0;
  double delta_r_rad = 0.0;
  convert_steering_angle(steering_angle_deg, delta_l_rad, delta_r_rad);
  

  // START Inicjalizacji zmiennych dla PID !!!!!!!!!

  uint16_t SWITCH_KEY_MOTION_MODES = get_parameter("switch_key_motion_modes").as_int();

  double gain_const = get_parameter("gain_const").as_double();
  double Kpro_fr = get_parameter("Kpro_fr").as_double() * gain_const;
  double Kpro_fl = get_parameter("Kpro_fl").as_double() * gain_const;
  double Kpro_rr = get_parameter("Kpro_rr").as_double() * gain_const;
  double Kpro_rl = get_parameter("Kpro_rl").as_double() * gain_const;
  double Kint_fr = get_parameter("Kint_fr").as_double() * gain_const;
  double Kint_fl = get_parameter("Kint_fl").as_double() * gain_const;
  double Kint_rr = get_parameter("Kint_rr").as_double() * gain_const;
  double Kint_rl = get_parameter("Kint_rl").as_double() * gain_const;
  
  double pedal_set_velocity = 0.0;

  // uint16_t SWITCH_KEY_MOTION_MODES = switch_key_motion_modes_;  // ros2 param set /controller switch_key_motion_modes 2
    // const uint16_t SWITCH_KEY_MOTION_MODES = 3;
  // OBECNA WARTOSC -> ros2 param get /controller switch_key_motion_modes 
  /*
  ZERO MODE = 0 (0 TORQUE, CAR DOESN'T MOVE)
  PEDAL MODE = 1 (THE SAME SPEED FOR EVERY WHEEL)
  MONO MODE = 2 (CONSTANT SPEED)
  SPEED VECTORING MODE = 3 (DURING TURNS BASED ON STEERING WHEEL PROPORTIONALLY SLOWS DOWN INNER WHEELS AND SPEEDS UP OUTER WHEELS)
  CZOLG MODE = 4 (TURNS LIKE A FUCKING TANK ITS CRAZY)
  */

  // Ograniczenia
  double LOW_CLAMP, HIGH_CLAMP, tau_signal_attempt;
  bool sat;


  // KONIEC inicjalizacji zmiennych dla PID!!!!!!!!!!

  double vx_est = 1.0;
  double vy_est = 0.0;
  estimate_velocity_ekf(ax, ay, yaw_rate, w_fl, w_fr, w_rl, w_rr, delta_l_rad, delta_r_rad, vx_est, vy_est);
  double delta_avg_rad = (delta_l_rad + delta_r_rad) / 2.0;
  double yaw_rate_ref = referenceYawRate(vx_est, delta_avg_rad * 180.0 / M_PI);
  double fz_fl = 0.0, fz_fr = 0.0, fz_rl = 0.0, fz_rr = 0.0;
  calculate_load_transfer(ax, ay, fz_fl, fz_fr, fz_rl, fz_rr);



    tau_final[0] = std::max(0.0,tau_prev[0]-MAX_DTAU);
    tau_final[1] = std::max(0.0,tau_prev[1]-MAX_DTAU);
    tau_final[2] = std::max(0.0,tau_prev[2]-MAX_DTAU);
    tau_final[3] = std::max(0.0,tau_prev[3]-MAX_DTAU);



    if(pedal <= 0.05){
    integral_front_left  = 0.0;
    integral_front_right = 0.0;
    integral_rear_left   = 0.0;
    integral_rear_right  = 0.0;
    }

    switch(SWITCH_KEY_MOTION_MODES){ // different modes for running the car on PID regulators :D
      break;
      case 1: 
          pedal_set_velocity = (pedal - 0.05)*30;
          velocity_set_front_left = pedal_set_velocity;
          velocity_set_front_right = pedal_set_velocity;
          velocity_set_rear_left = pedal_set_velocity;
          velocity_set_rear_right = pedal_set_velocity;

          // velocity_set_front_left = std::min(pedal_set_velocity,15);
          // velocity_set_front_right = std::min(pedal_set_velocity,15);
          // velocity_set_rear_left = std::min(pedal_set_velocity,15);
          // velocity_set_rear_right = std::min(pedal_set_velocity,15);
      break;
      case 2: 
          velocity_set_front_left = VELOCITY_SET_MONO_MODE;
          velocity_set_front_right = VELOCITY_SET_MONO_MODE;
          velocity_set_rear_left = VELOCITY_SET_MONO_MODE;
          velocity_set_rear_right = VELOCITY_SET_MONO_MODE;

      break;
      case 3: 
        pedal_set_velocity = (pedal - 0.05)*30;
        double DELTA_RAD = (0.00005 * steering_angle_deg * steering_angle_deg + 0.285758 * steering_angle_deg + 1.576273) * (M_PI / 180.0);
        double RADIOUS_MIDDLE = CAR_LENGTH/tan(DELTA_RAD);
        double RADIOUS_INNER = abs(RADIOUS_MIDDLE - CAR_WIDTH/2);
        double RADIOUS_OUTER = abs(RADIOUS_MIDDLE + CAR_WIDTH/2);

        velocity_set_front_left = std::max(0.0, abs(pedal_set_velocity * (RADIOUS_OUTER/RADIOUS_MIDDLE)));
        velocity_set_rear_left = std::max(0.0, abs(pedal_set_velocity * (RADIOUS_OUTER/RADIOUS_MIDDLE)));


        velocity_set_front_right = std::max(0.0, abs(pedal_set_velocity * (RADIOUS_INNER/RADIOUS_MIDDLE)));
        velocity_set_rear_right = std::max(0.0, abs(pedal_set_velocity * (RADIOUS_INNER/RADIOUS_MIDDLE)));

      break;
      default:
          velocity_set_front_left = 0.0;
          velocity_set_front_right = 0.0;
          velocity_set_rear_left = 0.0;
          velocity_set_rear_right = 0.0;
      break;}
    
        if(pedal > 0.05){

          // ---------- FL ----------
          auto velocity_front_left_error = velocity_set_front_left - (speed_fl * RPM_TO_MPS);

          LOW_CLAMP = std::max(TAU_MIN, tau_prev[0] - MAX_DTAU);
          HIGH_CLAMP = std::min(TAU_MAX, tau_prev[0] + MAX_DTAU);  


          tau_signal_attempt = Kpro_fl * velocity_front_left_error + Kint_fl * (integral_front_left + velocity_front_left_error * refresh_rate);
          sat = (tau_signal_attempt > HIGH_CLAMP && velocity_front_left_error > 0) || (tau_signal_attempt < LOW_CLAMP && velocity_front_left_error < 0);
          if (!sat) integral_front_left += velocity_front_left_error * refresh_rate;


          tau_final[0] = std::clamp(Kpro_fl * velocity_front_left_error + Kint_fl * integral_front_left, LOW_CLAMP, HIGH_CLAMP);
          // ---------- FR ----------
          auto velocity_front_right_error = velocity_set_front_right - (speed_fr * RPM_TO_MPS);

          LOW_CLAMP  = std::max(TAU_MIN, tau_prev[1] - MAX_DTAU);
          HIGH_CLAMP = std::min(TAU_MAX, tau_prev[1] + MAX_DTAU);

          tau_signal_attempt = Kpro_fr * velocity_front_right_error + Kint_fr * (integral_front_right + velocity_front_right_error * refresh_rate);
          sat = (tau_signal_attempt > HIGH_CLAMP && velocity_front_right_error > 0) || (tau_signal_attempt < LOW_CLAMP && velocity_front_right_error < 0);
          if (!sat) integral_front_right += velocity_front_right_error * refresh_rate;

          tau_final[1] = std::clamp(Kpro_fr * velocity_front_right_error + Kint_fr * integral_front_right, LOW_CLAMP, HIGH_CLAMP);

          // ---------- RL ----------
          auto velocity_rear_left_error = velocity_set_rear_left - (speed_rl * RPM_TO_MPS);

          LOW_CLAMP  = std::max(TAU_MIN, tau_prev[2] - MAX_DTAU);
          HIGH_CLAMP = std::min(TAU_MAX, tau_prev[2] + MAX_DTAU);

          tau_signal_attempt = Kpro_rl * velocity_rear_left_error + Kint_rl * (integral_rear_left + velocity_rear_left_error * refresh_rate);
          sat = (tau_signal_attempt > HIGH_CLAMP && velocity_rear_left_error > 0) || (tau_signal_attempt < LOW_CLAMP && velocity_rear_left_error < 0);
          if (!sat) integral_rear_left += velocity_rear_left_error * refresh_rate;

          tau_final[2] = std::clamp(Kpro_rl * velocity_rear_left_error + Kint_rl * integral_rear_left, LOW_CLAMP, HIGH_CLAMP);

          // ---------- RR ----------
          auto velocity_rear_right_error = velocity_set_rear_right - (speed_rr * RPM_TO_MPS);

          LOW_CLAMP  = std::max(TAU_MIN, tau_prev[3] - MAX_DTAU);
          HIGH_CLAMP = std::min(TAU_MAX, tau_prev[3] + MAX_DTAU);

          tau_signal_attempt = Kpro_rr * velocity_rear_right_error + Kint_rr * (integral_rear_right + velocity_rear_right_error * refresh_rate);
          sat = (tau_signal_attempt > HIGH_CLAMP && velocity_rear_right_error > 0) || (tau_signal_attempt < LOW_CLAMP && velocity_rear_right_error < 0);
          if (!sat) integral_rear_right += velocity_rear_right_error * refresh_rate;

          tau_final[3] = std::clamp(Kpro_rr * velocity_rear_right_error + Kint_rr * integral_rear_right, LOW_CLAMP, HIGH_CLAMP);
          }


          
        if(pedal > 0.05 && ODPOWIEDZ_SKOKOWA){
          tau_final[0] = std::min(TAU_SKOKOWA[0],tau_prev[0]+MAX_DTAU);
          tau_final[1] = std::min(TAU_SKOKOWA[1],tau_prev[1]+MAX_DTAU);
          tau_final[2] = std::min(TAU_SKOKOWA[2],tau_prev[2]+MAX_DTAU);
          tau_final[3] = std::min(TAU_SKOKOWA[3],tau_prev[3]+MAX_DTAU);
        }


  tau_prev[0] = tau_final[0];
  tau_prev[1] = tau_final[1];
  tau_prev[2] = tau_final[2];
  tau_prev[3] = tau_final[3];

  setpoints.front_left.torque = -tau_final[0];
  setpoints.front_right.torque = tau_final[1];
  setpoints.rear_left.torque = tau_final[2];
  setpoints.rear_right.torque = tau_final[3];

  setpoints_publisher->publish(setpoints);
}

inline double Controller::convert_pedal_position(int16_t pedal_position) {
  static constexpr double PEDAL_SCALER = 500.0;
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
  
  // Zawsze liczymy bezwzględny kąt na podstawie wielomianów
  double inner_wheel = 0.000410 * S_abs * S_abs + 0.2554 * S_abs;
  double outer_wheel = -0.0000942 * S_abs * S_abs + 0.2543 * S_abs;

  if (steering_wheel_deg >= 0.0) {
    // LEWO (Dodatnie) -> Lewe koło jest wewnętrzne
    delta_l_rad = inner_wheel;
    delta_r_rad = outer_wheel;
  } else {
    // PRAWO (Ujemne) -> Prawe koło jest wewnętrzne
    // UWAGA: Kąty muszą być ujemne dla skrętu w prawo!
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

  fz_fl = Fz_static_front - dFz_long + dFz_lat_front; 
  fz_fr = Fz_static_front - dFz_long - dFz_lat_front; 
  
  fz_rl = Fz_static_rear  + dFz_long + dFz_lat_rear;
  fz_rr = Fz_static_rear  + dFz_long - dFz_lat_rear;

  if (fz_fl < 10.0) fz_fl = 10.0;
  if (fz_fr < 10.0) fz_fr = 10.0;
  if (fz_rl < 10.0) fz_rl = 10.0;
  if (fz_rr < 10.0) fz_rr = 10.0;
}

inline int32_t Controller::convert_torque(double torque) {
  static constexpr double TORQUE_SCALER = 108.0;
  return (int32_t)((torque / 108.0) * TORQUE_SCALER);
}

inline void Controller::estimate_velocity_ekf(double ax, double ay, double r, double w_fl, double w_fr, double w_rl, double w_rr, double delta_l, double delta_r, double &vx_est, double &vy_est) {

    const double c   = 0.621;
    const double b   = 0.765;

    // Parametry 
    const double q_vx      = 0.206283;
    const double q_vy      = 0.00010;
    const double r_wheels  = 0.005777;
    const double r_yaw     = 0.007262;
    const double r_zlvu    = 0.00969129;
    const double ax_pen    = 0.006621;

    Eigen::Matrix2d Q;
    Q << q_vx, 0.0,
         0.0,  q_vy;

    double delta = (delta_l + delta_r) / 2.0;

    // PREDYKCJA
    Eigen::Vector2d x_pred;
    x_pred(0) = ekf_x(0) + (ax + ekf_x(1) * r) * dt;
    x_pred(1) = ekf_x(1) + (ay - ekf_x(0) * r) * dt;

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
        // Koła: twarde odcięcie przy poślizgu > 50%
        for (int i = 0; i < 4; i++) {
            double slip = 0.0;
            if (x_pred(0) > 1.0) {
                slip = std::abs(y_wheels(i)) / x_pred(0);
            }
            if (slip > 0.5) {
                R_adaptive(i, i) = 1e6;
            } else {
                R_adaptive(i, i) = r_wheels + std::abs(ax) * ax_pen;
            }
        }

        // Pomiar boczny
        if (is_straight) {
            z_full(4)    = 0.0;
            R_adaptive(4, 4) = r_zlvu;
        } else {
            z_full(4)    = r * b;
            R_adaptive(4, 4) = r_yaw + std::abs(ay) * 0.2;
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
