#include "ismpc_ros_client/mpc_interface.h"

#include <ismpc_cpp/representations/state.h>
#include <ismpc_cpp/types/configs.h>

namespace ismpc {
namespace ros {
MpcInterface::MpcInterface()
    : params_(),
      frame_info_(),
      reference_(params_),
      plan_(params_),
      state_(params_),
      planner_(frame_info_, reference_, state_, plan_, params_),
      mc_provider_(frame_info_, state_, plan_, params_),
      foot_trajectory_generator_(frame_info_, state_, plan_, params_) {}

void MpcInterface::step(const ismpc_interfaces::msg::LipData &lip_data) {
    // Foot Planning
    planner_.update(plan_);
    mc_provider_.update(plan_);
    foot_trajectory_generator_.update(state_);

    // MPC Execution

    // Advance time
    frame_info_.tk += params_.mpc.delta;
    frame_info_.k += 1;
}

// Additional methods for MpcInterface can be implemented here
}  // namespace ros
}  // namespace ismpc