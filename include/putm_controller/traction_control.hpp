#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace putm {

struct TcConfig {
  std::string algorithm = "legacy";

  // Geometry / kinematics
  double ratio     = 1.08;
  double radius    = 0.19796667568079587;
  double min_speed = 1.0;

  // Legacy
  double legacy_gain = 80.0;

  // PID
  double kp             = 500.0;
  double ki             = 1500.0;
  double kd             = 3.0;
  double derivative_tau = 0.03;

  // Wheel model / observer
  double inertia           = 0.45;
  double wheel_nm_per_unit = 1.078;
  double observer_tau      = 0.04;

  // Sliding mode
  double smc_gain     = 10.0;
  double smc_boundary = 2.0;

  void validate() const {
    if (algorithm != "legacy" && algorithm != "pid" && algorithm != "smc") {
      throw std::invalid_argument("tc_algorithm must be legacy, pid or smc");
    }

    for (double v : {ratio, radius, min_speed, legacy_gain,
                     kp, ki, kd, derivative_tau,
                     inertia, wheel_nm_per_unit,
                     smc_gain, smc_boundary, observer_tau}) {
      if (!std::isfinite(v) || v < 0) {
        throw std::invalid_argument("Invalid TC parameter");
      }
    }

    if (ratio <= 1 || radius <= 0 || min_speed <= 0 || derivative_tau <= 0 ||
        inertia <= 0 || wheel_nm_per_unit <= 0 || smc_boundary <= 0 ||
        observer_tau <= 0) {
      throw std::invalid_argument("TC scales/time constants must be positive");
    }
  }
};

class TractionControl {
 public:
  TcConfig config;

  void reset() { states_ = {}; }

  void set_previous_applied_torque(unsigned wheel, double torque) {
    if (!std::isfinite(torque)) {
      throw std::invalid_argument("Nonfinite applied torque");
    }
    states_.at(wheel).previous = torque;
  }

  double limit(unsigned wheel, double requested, double max_torque, double v,
               double omega, double acceleration, double dt, bool enabled) {
    auto &s = states_.at(wheel);

    if (!std::isfinite(requested) || !std::isfinite(v) ||
        !std::isfinite(omega) || !std::isfinite(acceleration) ||
        !std::isfinite(dt) || dt <= 0 || max_torque <= 0) {
      s = {};
      return 0.0;
    }

    const double demand = std::clamp(requested, 0.0, max_torque);

    if (!s.ready) {
      s.omega = omega;
      s.ready = true;
    }
    const double alpha = dt / (config.observer_tau + dt);
    const double load  = s.previous * config.wheel_nm_per_unit -
                         config.inertia * (omega - s.omega) / dt;
    s.load += alpha * (load - s.load);
    s.omega = omega;

    if (!enabled || requested <= 0 || v < config.min_speed) {
      s.integral   = 0;
      s.derivative = 0;
      s.error      = 0;
      s.active     = false;
      s.previous   = std::clamp(requested, -max_torque, max_torque);
      return s.previous;
    }

    const double error = (config.radius * omega - v) / v - (config.ratio - 1.0);
    double result = demand;

    if (config.algorithm == "legacy") {
      const double excess = std::max(0.0, config.radius * omega - config.ratio * v);
      result = std::min(demand,
                        std::max(0.0, max_torque - config.legacy_gain * excess));

    } else if (config.algorithm == "pid") {
      s.derivative += dt / (config.derivative_tau + dt) *
                      ((error - s.error) / dt - s.derivative);

      const double candidate = std::max(0.0, s.integral + config.ki * error * dt);
      const double reduction = config.kp * error + candidate + config.kd * s.derivative;

      // Conditional integration at both actuator limits; negative error unwinds.
      if (reduction <= demand || error < 0) {
        s.integral = std::min(candidate, max_torque);
      }

      const double correction = config.kp * error + s.integral +
                                config.kd * s.derivative;
      result = demand - std::clamp(correction, 0.0, demand);

    } else {  // smc
      // S = omega - ratio*v/R, Sdot = (T-load)/J - ratio*a/R.
      // Equivalent torque is derived directly from wheel dynamics; softened
      // switching follows Alfoldi et al. (2024), adapted to a unilateral ceiling.
      const double surface = omega - config.ratio * v / config.radius;
      const double equivalent =
          s.load + config.inertia * config.ratio * acceleration / config.radius;
      const double cap =
          (equivalent - config.smc_gain * surface /
                            (std::abs(surface) + config.smc_boundary)) /
          config.wheel_nm_per_unit;

      // Below ceiling pass demand. Never command additional drive or braking.
      if (surface > 0) s.active = true;
      if (surface < -config.smc_boundary) s.active = false;
      if (s.active) result = std::min(demand, std::max(0.0, cap));
    }

    s.error    = error;
    s.previous = std::clamp(result, 0.0, demand);
    return s.previous;
  }

 private:
  struct State {
    double integral   = 0;
    double derivative = 0;
    double error      = 0;
    double omega      = 0;
    double load       = 0;
    double previous   = 0;
    bool   ready      = false;
    bool   active     = false;
  };

  std::array<State, 4> states_{};
};

}  // namespace putm