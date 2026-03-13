#ifndef STEPIT_POLICY_H_
#define STEPIT_POLICY_H_

#include <stepit/communication.h>
#include <stepit/control_input.h>
#include <stepit/robot.h>

namespace stepit {
struct PolicySpec : RobotSpec {
  explicit PolicySpec(const RobotSpec &robot_spec, std::string home_dir)
      : RobotSpec(robot_spec), home_dir(std::move(home_dir)) {}

  /** Home directory from which the policy reads configuration files. */
  std::string home_dir;
  /** Policy name. */
  std::string policy_name;
  /** Control frequency in Hz. */
  std::size_t control_freq{};
  /** Whether the policy remains trusted when safety violations occur. */
  bool trusted{};
};

class Policy : public Interface<Policy, const RobotSpec & /* robot_spec */, const std::string & /* home_dir */> {
 public:
  virtual const PolicySpec &getSpec() const = 0;
  std::string getHomeDir() const { return getSpec().home_dir; }
  std::size_t getControlFreq() const { return getSpec().control_freq; }
  std::string getName() const { return getSpec().policy_name; }
  bool isTrusted() const { return getSpec().trusted; }

  /**
   * Resets the policy to its initial state.
   *
   * @return True if reset succeeded; otherwise, false.
   */
  virtual bool reset() = 0;
  /**
   * Produces a control action from the current low-level state.
   *
   * @param low_state Current low-level state information used for the control decision.
   * @param requests Current control requests to be handled by the policy.
   * @param cmd Output parameter populated with the resulting low-level command.
   * @return True if the action was computed successfully; otherwise, false.
   */
  virtual bool act(const LowState &low_state, ControlRequests &requests, LowCmd &cmd) = 0;
  /**
   * Runs an optional post-act phase after the latest low-level command is handed off.
   *
   * Use this phase for non-critical work such as state publication so the control path can prioritize
   * `act()` and low-level command updates.
   */
  virtual void postAct() {}
  /**
   * Shuts down the policy and releases any owned resources.
   */
  virtual void exit() = 0;
};
}  // namespace stepit

#define STEPIT_REGISTER_POLICY(name, priority, factory) \
  static ::stepit::Policy::Registration _policy_##name##_registration(#name, priority, factory)

#endif  // STEPIT_POLICY_H_
