#ifndef STEPIT_ROBOT_UNITREE_H_
#define STEPIT_ROBOT_UNITREE_H_

#include <unitree_legged_sdk/unitree_legged_sdk.h>

#include <memory>

#include <stepit/robot.h>

#define UNITREE_ALIENGO 0
#define UNITREE_GO1     1
#define UNITREE_B1      2

namespace stepit {
namespace unitree = UNITREE_LEGGED_SDK;

class UnitreeApi final : public RobotApi {
 public:
  UnitreeApi();
  ~UnitreeApi() override;
  void getControl(bool enable) override;
  void setSend(LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override;
  void recv() override;

  static constexpr const char *getRobotName();

 private:
#if STEPIT_UNITREE_ROBOT == UNITREE_ALIENGO
  class RawUdp;
  std::unique_ptr<RawUdp> raw_udp_;
#endif
  std::unique_ptr<unitree::UDP> udp_;
  bool use_raw_udp_{false};
  unitree::LowCmd low_cmd_{};
  unitree::LowState low_state_{};
};
}  // namespace stepit

#endif  // STEPIT_ROBOT_UNITREE_H_
