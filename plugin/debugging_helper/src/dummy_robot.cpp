#include <stepit/debugging/dummy_robot.h>

namespace stepit {
DummyRobotApi::DummyRobotApi() : RobotApi(kRobotName), low_state_(getDoF(), getNumLegs()), start_time_(Clock::now()) {}

void DummyRobotApi::setSend(LowCmd &cmd) {
  for (std::size_t i{}; i < getDoF(); ++i) {
    low_state_.motor_state[i].q   = cmd[i].q;
    low_state_.motor_state[i].dq  = cmd[i].dq;
    low_state_.motor_state[i].tor = cmd[i].tor;
  }
}

void DummyRobotApi::getRecv(LowState &state) {
  low_state_.tick = static_cast<uint32_t>(duration_cast<MSec>(Clock::now() - start_time_).count());
  state           = low_state_;
}

STEPIT_REGISTER_ROBOTAPI(dummy, kMinPriority, [] { return std::make_unique<DummyRobotApi>(); });
}  // namespace stepit
