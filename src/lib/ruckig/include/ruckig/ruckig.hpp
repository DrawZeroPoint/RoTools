#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <tuple>

#include <ruckig/calculator.hpp>
#include <ruckig/input_parameter.hpp>
#include <ruckig/output_parameter.hpp>
#include <ruckig/trajectory.hpp>

namespace ruckig {

constexpr static size_t kDynamicDoFs{0};

//! Main class for the Ruckig algorithm.
template <size_t DOFs = 0, bool throw_error = false, bool return_error_at_maximal_duration = true>
class Ruckig {
  //! Current input, only for comparison for recalculation
  InputParameter<DOFs> current_input_;

  //! Flag that indicates if the current_input was properly initialized
  bool current_input_initialized_{false};

  //! Calculator for new trajectories
  Calculator<DOFs> calculator_;

  //! Max number of intermediate waypoints
  const size_t max_number_of_waypoints;

  inline static double vAtAZero(double v0, double a0, double j) { return v0 + (a0 * a0) / (2 * j); }

 public:
  size_t degrees_of_freedom;

  //! Time step between updates (cycle time) in [s]
  const double delta_time{0.0};

  template <size_t D = DOFs, typename std::enable_if<D >= 1, int>::type = 0>
  explicit Ruckig() : degrees_of_freedom(DOFs), delta_time(-1.0), max_number_of_waypoints(0) {}

  template <size_t D = DOFs, typename std::enable_if<D >= 1, int>::type = 0>
  explicit Ruckig(double delta_time) : degrees_of_freedom(DOFs), delta_time(delta_time), max_number_of_waypoints(0) {}

#if defined WITH_ONLINE_CLIENT
  template <size_t D = DOFs, typename std::enable_if<D >= 1, int>::type = 0>
  explicit Ruckig(double delta_time, size_t max_number_of_waypoints)
      : degrees_of_freedom(DOFs),
        delta_time(delta_time),
        calculator(Calculator<DOFs>(max_number_of_waypoints)),
        max_number_of_waypoints(max_number_of_waypoints),
        current_input(InputParameter<DOFs>(max_number_of_waypoints)) {}
#endif

  template <size_t D = DOFs, typename std::enable_if<D == 0, int>::type = 0>
  explicit Ruckig(size_t dofs)
      : degrees_of_freedom(dofs),
        delta_time(-1.0),
        calculator_(Calculator<0>(dofs)),
        max_number_of_waypoints(0),
        current_input_(InputParameter<0>(dofs)) {}

  template <size_t D = DOFs, typename std::enable_if<D == 0, int>::type = 0>
  explicit Ruckig(size_t dofs, double delta_time)
      : degrees_of_freedom(dofs),
        delta_time(delta_time),
        calculator_(Calculator<0>(dofs)),
        max_number_of_waypoints(0),
        current_input_(InputParameter<0>(dofs)) {}

#if defined WITH_ONLINE_CLIENT
  template <size_t D = DOFs, typename std::enable_if<D == 0, int>::type = 0>
  explicit Ruckig(size_t dofs, double delta_time, size_t max_number_of_waypoints)
      : degrees_of_freedom(dofs),
        delta_time(delta_time),
        calculator(Calculator<0>(dofs, max_number_of_waypoints)),
        max_number_of_waypoints(max_number_of_waypoints),
        current_input(InputParameter<0>(dofs, max_number_of_waypoints)) {}
#endif

  //! Filter intermediate positions based on a threshold distance for each DoF
  template <class T>
  using Vector = typename std::conditional<DOFs >= 1, std::array<T, DOFs>, std::vector<T>>::type;
  std::vector<Vector<double>> filterIntermediatePositions(const InputParameter<DOFs>& input,
                                                          const Vector<double>& threshold_distance) const {
    if (input.intermediate_positions.empty()) {
      return input.intermediate_positions;
    }

    const size_t kNWaypoints = input.intermediate_positions.size();
    std::vector<bool> is_active;
    is_active.resize(kNWaypoints);
    for (size_t i = 0; i < kNWaypoints; ++i) {
      is_active[i] = true;
    }

    size_t start = 0;
    size_t end = start + 2;
    for (; end < kNWaypoints + 2; ++end) {
      const auto kPosStart = (start == 0) ? input.current_position : input.intermediate_positions[start - 1];
      const auto kPosEnd = (end == kNWaypoints + 1) ? input.target_position : input.intermediate_positions[end - 1];

      // Check for all intermediate positions
      bool are_all_below{true};
      for (size_t current = start + 1; current < end; ++current) {
        const auto kPosCurrent = input.intermediate_positions[current - 1];

        // Is there a point t on the line that holds the threshold?
        double t_start_max = 0.0;
        double t_end_min = 1.0;
        for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
          const double kH0 = (kPosCurrent[dof] - kPosStart[dof]) / (kPosEnd[dof] - kPosStart[dof]);
          const double kTStart = kH0 - threshold_distance[dof] / std::abs(kPosEnd[dof] - kPosStart[dof]);
          const double kTEnd = kH0 + threshold_distance[dof] / std::abs(kPosEnd[dof] - kPosStart[dof]);

          t_start_max = std::max(kTStart, t_start_max);
          t_end_min = std::min(kTEnd, t_end_min);

          if (t_start_max > t_end_min) {
            are_all_below = false;
            break;
          }
        }
        if (!are_all_below) {
          break;
        }
      }

      is_active[end - 2] = !are_all_below;
      if (!are_all_below) {
        start = end - 1;
      }
    }

    std::vector<Vector<double>> filtered_positions;
    filtered_positions.reserve(kNWaypoints);
    for (size_t i = 0; i < kNWaypoints; ++i) {
      if (is_active[i]) {
        filtered_positions.push_back(input.intermediate_positions[i]);
      }
    }

    return filtered_positions;
  }

  //! Validate the input for trajectory calculation and kinematic limits
  bool validateInput(const InputParameter<DOFs>& input,
                     bool check_current_state_within_limits = false,
                     bool check_target_state_within_limits = true) const {
    for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
      const double kJMax = input.max_jerk[dof];
      if (std::isnan(kJMax) || kJMax <= std::numeric_limits<double>::min()) {
        return false;
      }

      const double kAMax = input.max_acceleration[dof];
      if (std::isnan(kAMax) || kAMax <= std::numeric_limits<double>::min()) {
        return false;
      }

      const double kAMin = input.min_acceleration ? input.min_acceleration.value()[dof] : -input.max_acceleration[dof];
      if (std::isnan(kAMin) || kAMin >= -std::numeric_limits<double>::min()) {
        return false;
      }

      const double kA0 = input.current_acceleration[dof];
      const double kAf = input.target_acceleration[dof];
      if (std::isnan(kA0) || std::isnan(kAf)) {
        return false;
      }

      if (check_current_state_within_limits && (kA0 > kAMax || kA0 < kAMin)) {
        return false;
      }
      if (check_target_state_within_limits && (kAf > kAMax || kAf < kAMin)) {
        return false;
      }

      const double kV0 = input.current_velocity[dof];
      const double kVf = input.target_velocity[dof];
      if (std::isnan(kV0) || std::isnan(kVf)) {
        return false;
      }

      auto control_interface =
          input.per_dof_control_interface ? input.per_dof_control_interface.value()[dof] : input.control_interface;
      if (control_interface == ControlInterface::Position) {
        if (std::isnan(input.current_position[dof]) || std::isnan(input.target_position[dof])) {
          return false;
        }

        const double kVMax = input.max_velocity[dof];
        if (std::isnan(kVMax) || kVMax <= std::numeric_limits<double>::min()) {
          return false;
        }

        const double kVMin = input.min_velocity ? input.min_velocity.value()[dof] : -input.max_velocity[dof];
        if (std::isnan(kVMin) || kVMin >= -std::numeric_limits<double>::min()) {
          return false;
        }

        if (check_current_state_within_limits && (kV0 > kVMax || kV0 < kVMin)) {
          return false;
        }
        if (check_target_state_within_limits && (kVf > kVMax || kVf < kVMin)) {
          return false;
        }

        if (check_current_state_within_limits &&
            ((kA0 > 0 && vAtAZero(kV0, kA0, kJMax) > kVMax) || (kA0 < 0 && vAtAZero(kV0, kA0, -kJMax) < kVMin))) {
          return false;
        }
        if (check_target_state_within_limits &&
            ((kAf < 0 && vAtAZero(kVf, kAf, kJMax) > kVMax) || (kAf > 0 && vAtAZero(kVf, kAf, -kJMax) < kVMin))) {
          return false;
        }
      }
    }

    if (!input.intermediate_positions.empty() && input.control_interface == ControlInterface::Position) {
      if (input.intermediate_positions.size() > max_number_of_waypoints) {
        return false;
      }

      if (input.minimum_duration || input.duration_discretization != DurationDiscretization::Continuous) {
        return false;
      }

      if (input.per_dof_control_interface || input.per_dof_synchronization) {
        return false;
      }
    }

    if (delta_time <= 0.0 && input.duration_discretization != DurationDiscretization::Continuous) {
      return false;
    }

    return true;
  }

  //! Calculate a new trajectory for the given input
  Result calculate(const InputParameter<DOFs>& input, Trajectory<DOFs>& trajectory) {
    bool was_interrupted{false};
    return calculate(input, trajectory, was_interrupted);
  }

  //! Calculate a new trajectory for the given input and check for interruption
  Result calculate(const InputParameter<DOFs>& input, Trajectory<DOFs>& trajectory, bool& was_interrupted) {
    if (!validateInput(input, false, true)) {
      return Result::ErrorInvalidInput;
    }

    return calculator_.template calculate<throw_error, return_error_at_maximal_duration>(input, trajectory, delta_time,
                                                                                         was_interrupted);
  }

  //! Get the next output state (with step delta_time) along the calculated trajectory for the given input
  Result update(const InputParameter<DOFs>& input, OutputParameter<DOFs>& output) {
    const auto kStart = std::chrono::high_resolution_clock::now();

    if constexpr (DOFs == 0 && throw_error) {
      if (degrees_of_freedom != input.degrees_of_freedom || degrees_of_freedom != output.degrees_of_freedom) {
        throw std::runtime_error("[ruckig] mismatch in degrees of freedom (vector size).");
      }
    }

    output.new_calculation = false;

    if (input != current_input_ || !current_input_initialized_) {
      Result result = calculate(input, output.trajectory, output.was_calculation_interrupted);
      if (result != Result::Working) {
        return result;
      }

      current_input_ = input;
      current_input_initialized_ = true;
      output.time = 0.0;
      output.new_calculation = true;
    }

    const size_t kOldSection = output.new_section;
    output.time += delta_time;
    output.trajectory.at_time(output.time, output.new_position, output.new_velocity, output.new_acceleration,
                              output.new_section);
    output.did_section_change = (output.new_section != kOldSection);

    const auto kStop = std::chrono::high_resolution_clock::now();
    output.calculation_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(kStop - kStart).count() / 1000.0;

    output.pass_to_input(current_input_);

    if (output.time > output.trajectory.get_duration()) {
      return Result::Finished;
    }

    return Result::Working;
  }
};

}  // namespace ruckig
