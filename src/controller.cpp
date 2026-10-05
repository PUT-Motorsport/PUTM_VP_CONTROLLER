#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/bms_hv_main.hpp"
#include "putm_vcl_interfaces/msg/setpoints.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values1.hpp"
#include "putm_vcl_interfaces/msg/steering_wheel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "putm_vcl_interfaces/msg/yaw_ref.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

// #include "putm_vcl_interfaces/msg/xsens_acceleration.hpp"
// #include "putm_vcl_interfaces/msg/xsens_rate_of_turn.hpp"
// #include "vectornav_msgs/msg/imu_group.hpp"

extern "C" {
#include "read.h"
#include "tv_code.h"
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

  double torque_fl;
  double torque_fr;
  double torque_rl;
  double torque_rr;

  rclcpp::QoS qos_;

  rclcpp::Time last_call_time_;

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

  rclcpp::Publisher<Setpoints>::SharedPtr setpoints_publisher;
  rclcpp::Publisher<YawRef>::SharedPtr yaw_rate_ref_publisher;
  rclcpp::Subscription<FrontboxDriverInput>::SharedPtr frontbox_driver_input_subscriber;
  rclcpp::Subscription<SteeringWheel>::SharedPtr steering_wheel_subscriber;
  rclcpp::TimerBase::SharedPtr control_loop_timer;
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

  inline double convert_pedal_position(int16_t pedal_position);
  inline int32_t convert_torque(double torque);
  inline double convert_wheel_speed(double rpm);
  inline void convert_steering_angle(double steering_wheel_deg, double &delta_l_rad, double &delta_r_rad);

  double speed_fl, speed_fr, speed_rl, speed_rr;
  double ay, ax, yaw_rate, batt_curr;
  Setpoints setpoints;

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

      qos_(rclcpp::QoS(1)
              .best_effort()
              .durability_volatile()),
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
      speed_fl(0), speed_fr(0), speed_rl(0), speed_rr(0),
      control_loop_timer(this->create_wall_timer(10ms, std::bind(&Controller::control_loop, this))),
      ay(0.0), ax(0.0), yaw_rate(0.0), batt_curr(0.0)
      {
        rclcpp::QoS qos(1);
        qos.best_effort();
        qos.durability_volatile();
        last_call_time_ = this->now();
        tv_code_initialize();
        read_inputs();
        torque_fl = 0;
        torque_fr = 0;
        torque_rl = 0;
        torque_rr = 0;
      }

Controller::~Controller() { tv_code_terminate(); }

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
  if (rtmGetErrorStatus(tv_code_M) == (NULL) && !rtmGetStopRequested(tv_code_M)) {
    
    tv_code_P.acc_pedal_Value = convert_pedal_position(frontbox_driver_input.pedal_position);

    tv_code_P.delta_Value = ((double)steering_wheel.steering_wheel_position /135 * 50) * -1;


    tv_code_P.avg_min_speed_switch_CurrentSet = 1;


    tv_code_P.whl_speed_fl_Value = speed_rl;
    tv_code_P.whl_speed_fr_Value = speed_fr;
    tv_code_P.whl_speed_rl_Value = speed_rl;
    tv_code_P.whl_speed_rr_Value = speed_rr;

    // tv_code_P.whl_speed_fl_Value = 1000;
    // tv_code_P.whl_speed_fr_Value = 1000;
    // tv_code_P.whl_speed_rl_Value = 1000;
    // tv_code_P.whl_speed_rr_Value = 1000;

    tv_code_P.speed_switch_Threshold = 0;

    tv_code_P.TT_max_Value = 30;

    tv_code_P.regen_switch_CurrentSetting = 1;
    // tv_code_P.batt_curr_Value = abs(batt_curr/100);
    tv_code_P.yaw_rate_Value = yaw_rate;
    tv_code_P.ax_Value = ax;
    tv_code_P.ay_Value = ay;
    tv_code_P.Mz_p=300;
    tv_code_P.Mz_I=30;
    tv_code_P.Ku=-1/150;
    // tv_code_P.power_speed_limiter_switch_Thre = 100000000;
    
    tv_code_step();

    torque_fl = tv_code_P.acc_pedal_Value;
    torque_fr = tv_code_P.acc_pedal_Value;
    torque_rl = tv_code_P.acc_pedal_Value;
    torque_rr = tv_code_P.acc_pedal_Value;

    // torque_fl = tv_code_B.trq_fl / tv_code_P.drive_ratio ;
    // torque_fr = tv_code_B.trq_fr / tv_code_P.drive_ratio;
    // torque_rl = tv_code_B.trq_rl / tv_code_P.drive_ratio ;
    // torque_rr = tv_code_B.trq_rr / tv_code_P.drive_ratio ;

    // torque_fl/=tv_code_P.max_moment;
    // torque_fr/=tv_code_P.max_moment;
    // torque_rl/=tv_code_P.max_moment;
    // torque_rr/=tv_code_P.max_moment;

    


    auto setpoints = Setpoints();
    auto vpdata = YawRef();
    vpdata.yaw_rate_ref = tv_code_B.Saturation_j;
    // vpdata.est_power = tv_code_B.est_power;
    // vpdata.torque_fixed = tv_code_B.torque_fixed;
    // vpdata.ifl = tv_code_B.T_max;
    // vpdata.ufl = tv_code_B.UFL;
    // vpdata.ifr = tv_code_B.IFR;
    // vpdata.ufr = tv_code_B.UFR;
    // vpdata.irl = tv_code_B.IRL;
    // vpdata.url = tv_code_B.URL;
    // vpdata.irr = tv_code_B.IRR;
    // vpdata.urr = tv_code_B.URR;

    setpoints.front_left.torque = convert_torque(torque_fl)* -1;
    setpoints.front_right.torque = convert_torque(torque_fr);
    setpoints.rear_left.torque = convert_torque(torque_rl);
    setpoints.rear_right.torque = convert_torque(torque_rr);


    setpoints_publisher->publish(setpoints);
    yaw_rate_ref_publisher->publish(vpdata);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error in Simulink model");
  }
}

inline double Controller::convert_pedal_position(int16_t pedal_position) {
  static constexpr double PEDAL_SCALER = 500.0;
  return (((double)pedal_position) / PEDAL_SCALER);
}


inline int32_t Controller::convert_torque(double torque) {
  static constexpr double TORQUE_SCALER = 1000.0;
  return (int32_t)(torque * TORQUE_SCALER);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
}