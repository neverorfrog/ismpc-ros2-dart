#pragma once

#include "ismpc_cpp/modules/foot_trajectory_generator.h"
#include "ismpc_cpp/modules/footstep_plan_provider.h"
#include "ismpc_cpp/modules/moving_constraint_provider.h"
#include "ismpc_cpp/representations/footstep_plan.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/reference.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/types/configs.h"
#include "ismpc_interfaces/msg/lip_data.hpp"

namespace ismpc {
namespace ros {

class MpcInterface {
public:
  MpcInterface();

  const Params &getParams() const { return params_; }

  const FrameInfo &getFrameInfo() const { return frame_info_; }

  const Reference &getReference() const { return reference_; }

  const FootstepPlan &getFootstepPlan() const { return plan_; }

  const State &getState() const { return state_; }

  /**
   * @brief Perform a single step of the MPC.
   * This method updates the internal state of the MPC based on the current
   * frame information.
   * It computes the footstep plan and updates the state representation.
   * The state is updated periodically (simulation delta) using a listener.
   */
  void step(const ismpc_interfaces::msg::LipData &lip_data);

  /**
   * @brief Set the MPC state from the provided sim data.
   * @param sim_data The simulation data to set the MPC state from.
   */
  // void setStateFromSimData(const SimData &sim_data);

private:
  Params params_;

  // Representations
  FrameInfo frame_info_;
  Reference reference_;
  FootstepPlan plan_;
  State state_;

  // Modules
  FootstepPlanProvider planner_;
  MovingConstraintProvider mc_provider_;
  FootTrajectoryGenerator foot_trajectory_generator_;
};

} // namespace ros
} // namespace ismpc