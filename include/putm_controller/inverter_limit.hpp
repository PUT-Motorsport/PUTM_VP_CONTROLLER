#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "putm_controller/amk_map.hpp"
namespace putm {
// A2370DD datasheet: Mn=9.8 Nm, Mmax=21 Nm, Nmax=20000 rpm.
struct InverterLimit {
  double nominal_nm = 9.8;
  double peak_nm = 21.0;
  double max_rpm = 20000.0;
  double margin = 0.9;
  void validate() const {
    if (!std::isfinite(nominal_nm) || nominal_nm <= 0 ||
        !std::isfinite(peak_nm) || peak_nm <= 0 || peak_nm > 21 ||
        !std::isfinite(max_rpm) || max_rpm <= 0 || max_rpm > 20000 ||
        !std::isfinite(margin) || margin <= 0 || margin >= 1)
      throw std::invalid_argument("Invalid AMK limiter configuration");
  }
  double row_limit(unsigned row, double voltage) const {
    double torque = 0;
    for (unsigned col = 1; col < 21; ++col)
      if (amk_map::voltage_rms[row][col] <= voltage / std::sqrt(2.0))
        torque = std::max(torque, amk_map::torque_nm[row][col]);
    return std::min(peak_nm, torque);
  }
  double limit(double requested_percent, double rpm, double voltage, bool fresh) const {
    if (!fresh || !std::isfinite(requested_percent) || !std::isfinite(rpm) ||
        !std::isfinite(voltage) || voltage < 250 || voltage > 720 ||
        std::abs(rpm) >= max_rpm) return 0;
    // No generator map supplied: no unvalidated symmetry assumption for regen.
    if (requested_percent <= 0) return 0;
    voltage = std::min(voltage, 600.0); // map base voltage: never extrapolate upwards
    const double position = std::abs(rpm)/100.0;
    const unsigned lo = static_cast<unsigned>(std::floor(position));
    const unsigned hi = static_cast<unsigned>(std::ceil(position));
    // Lower adjacent speed envelope avoids optimistic interpolation.
    const double nm = std::min(row_limit(lo, voltage), row_limit(hi, voltage));
    return std::min(requested_percent, margin*100.0*nm/nominal_nm);
  }
};
}
