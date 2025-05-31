#include "ismpc_ros_client/mpc_interface.h"
#include <ismpc_cpp/representations/state.h>
#include <ismpc_cpp/types/configs.h>

namespace ismpc {
namespace ros {
MpcInterface::MpcInterface()
    : params_(), frame_info_(), reference_(params_), fs_plan_(params_),
      state_(params_) {}

void MpcInterface::mpcStep() {
  frame_info_.tk += 0.01;
  frame_info_.k += 1;
}

// Additional methods for MpcInterface can be implemented here
} // namespace ros
} // namespace ismpc