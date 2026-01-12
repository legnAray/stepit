#ifndef STEPIT_DEBUGGING_DUMMY_ROBOT_H_
#define STEPIT_DEBUGGING_DUMMY_ROBOT_H_

#include <stepit/robot.h>

namespace stepit {
class DummyRobotApi final : public RobotApi {
 public:
  DummyRobotApi();
  void getControl(bool enable) override {}
  void setSend(LowCmd &cmd) override;
  void getRecv(LowState &state) override;
  void send() override {}
  void recv() override {}

  static constexpr const char *kRobotName = "dummy";
  std::size_t getDoF() const override { return getSpec().dof; }
  std::size_t getNumLegs() const override { return getSpec().foot_names.size(); }
  std::size_t getCommFreq() const override { return 1000; }

 private:
  LowState low_state_;
  TimePoint start_time_;
};
}  // namespace stepit

#endif  // STEPIT_DEBUGGING_DUMMY_ROBOT_H_
