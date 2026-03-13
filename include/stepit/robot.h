#ifndef STEPIT_ROBOT_H_
#define STEPIT_ROBOT_H_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <stepit/registry.h>
#include <stepit/utils.h>

namespace stepit {
/** Feedback state of one motor. */
struct MotorState {
  /** Current joint angle in radians. */
  float q;
  /** Current joint velocity in radians per second. */
  float dq;
  /** Current estimated output torque in newton-meters. */
  float tor;
};

/** Whole-body low-level state feedback from the robot. */
struct LowState {
  LowState() = default;
  LowState(std::size_t dof, std::size_t num_legs) : motor_state(dof), foot_force(num_legs) {}

  struct IMU {
    /** Orientation quaternion in `(w, x, y, z)` order. */
    std::array<float, 4> quaternion{1.0, 0.0, 0.0, 0.0};
    /** Angular velocity in radians per second. */
    std::array<float, 3> gyroscope{};
    /** Linear acceleration in meters per second squared. */
    std::array<float, 3> accelerometer{};
    /** Euler angles `(roll, pitch, yaw)` in radians. */
    std::array<float, 3> rpy{};
  } imu;

  /** Whole-body joint states. */
  std::vector<MotorState> motor_state;
  /** Force sensed at each foot. */
  std::vector<float> foot_force;
  /** System tick count. */
  uint32_t tick{};
};

/** Control command for one motor. */
struct MotorCmd {
  /** Target joint angle in radians. */
  float q;
  /** Target joint velocity in radians per second. */
  float dq;
  /** Target feedforward torque in newton-meters. */
  float tor;
  /** Proportional gain in newton-meters per radian. */
  float Kp;
  /** Derivative gain in newton-meters per (radian per second). */
  float Kd;
};

/** Whole-body low-level command sent to the robot. */
using LowCmd = std::vector<MotorCmd>;

struct RobotSpec {
  RobotSpec() = default;
  explicit RobotSpec(const yml::Node &config);

  /** Robot name. */
  std::string robot_name;
  /** Joint names. */
  std::vector<std::string> joint_names;
  /** End-effector names for the feet. */
  std::vector<std::string> foot_names;
  /** Number of degrees of freedom. */
  std::size_t dof{};
  /** Number of legs. */
  std::size_t num_legs{};
  /** Communication frequency in Hz. */
  std::size_t comm_freq{};

  /** Proportional gains used for following fixed trajectories. */
  std::vector<float> kp;
  /** Derivative gains used for following fixed trajectories. */
  std::vector<float> kd;
  /** Maximum allowed joint position deviation when following fixed trajectories, in radians. */
  std::vector<float> stuck_threshold;

  struct Safety {
    /** Whether to freeze the robot when safety violations are detected. */
    bool enabled{true};
    /** Maximum allowed roll angle in radians. */
    float roll{M_PIf / 2};
    /** Maximum allowed pitch angle in radians. */
    float pitch{M_PIf / 2};
  } safety;

  float resetting_time{0.8};
  float standing_up_time{1.2};
  float lying_down_time{1.0};
  float returning_to_standing_time{1.0};
  std::vector<float> standing_cfg;
  std::vector<float> lying_cfg;
  /** Whether the robot automatically enters damped mode when low commands are not published. */
  bool auto_damped_mode{true};
  /** Damping coefficient used in damped mode when `auto_damped_mode` is false. */
  float kd_damped_mode{5.};
};

class RobotApi : public Interface<RobotApi> {
 public:
  RobotApi() = default;
  explicit RobotApi(const std::string &name)
      : config_(loadGlobalConfigYaml(fmt::format("robot/{}.yml", name))), spec_(config_) {}

  virtual void getControl(bool enable) = 0;
  virtual void setSend(const LowCmd &) = 0;
  virtual void getRecv(LowState &)     = 0;
  virtual void send()                  = 0;
  virtual void recv()                  = 0;

  const RobotSpec &getSpec() const { return spec_; }
  std::size_t getDoF() const { return spec_.dof; }
  std::size_t getNumLegs() const { return spec_.num_legs; }
  std::size_t getCommFreq() const { return spec_.comm_freq; }

 protected:
  yml::Node config_;
  RobotSpec spec_;
};

class RobotApiReorderingWrapper : public RobotApi {
 public:
  /**
   * Wraps another RobotApi instance and reorders joints and feet.
   *
   * @param exposed_name Name of this wrapper in the exposed spec.
   * @param wrapped_name Registered name of the underlying RobotApi implementation.
   * @param joint_order Mapping from wrapper joint index to wrapped joint index. Must be a permutation of size `dof`.
   * @param foot_order Mapping from wrapper foot index to wrapped foot index. If empty, identity mapping is used.
   * @param joint_reversed Per-joint direction flags. When `true`, position, velocity, and torque signs are flipped
   * for that joint during remapping. If empty, all joints are treated as non-reversed.
   */
  RobotApiReorderingWrapper(const std::string &exposed_name, const std::string &wrapped_name,
                            std::vector<std::size_t> joint_order, std::vector<std::size_t> foot_order = {},
                            std::vector<bool> joint_reversed = {});

  void getControl(bool enable) override { wrapped_->getControl(enable); }
  void setSend(const LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override { wrapped_->send(); }
  void recv() override { wrapped_->recv(); }

 private:
  static void validateOrder(const std::vector<std::size_t> &order, std::size_t expected_size, const char *name);

  std::unique_ptr<RobotApi> wrapped_;
  std::vector<std::size_t> joint_order_;
  std::vector<std::size_t> foot_order_;
  std::vector<bool> joint_reversed_;
};
}  // namespace stepit

#define STEPIT_REGISTER_ROBOTAPI(name, priority, factory) \
  static ::stepit::RobotApi::Registration _robotapi_##name##_registration(#name, priority, factory)

#endif  // STEPIT_ROBOT_H_
