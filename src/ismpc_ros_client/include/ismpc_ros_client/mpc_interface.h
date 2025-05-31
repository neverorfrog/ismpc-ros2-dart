#pragma once

#include "ismpc_cpp/representations/footstep_plan.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/reference.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/types/configs.h"

namespace ismpc {
namespace ros {

class MpcInterface {
public:
  MpcInterface();

  const FrameInfo &getFrameInfo() const { return frame_info_; }

  const Reference &getReference() const { return reference_; }

  const FootstepPlan &getFootstepPlan() const { return fs_plan_; }

  const State &getState() const { return state_; }

  /**
   * @brief Perform a single step of the MPC.
   * This method updates the internal state of the MPC based on the current
   * frame information.
   * TODO: Should actually return the state maybe?
   */
  void mpcStep();

private:
  Params params_;
  FrameInfo frame_info_;
  Reference reference_;
  FootstepPlan fs_plan_;
  State state_;
};

} // namespace ros
} // namespace ismpc