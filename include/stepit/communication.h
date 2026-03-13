#ifndef STEPIT_COMMUNICATION_H_
#define STEPIT_COMMUNICATION_H_

#include <pthread.h>
#include <sys/types.h>

#include <atomic>
#include <mutex>
#include <thread>

#include <stepit/publisher.h>

namespace stepit {
/**
 * Sets the CPU affinity for a specific thread or process.
 *
 * @param pid Process ID or thread ID to configure. If 0, the calling thread is used.
 * @param cpuid Logical CPU ID to bind the thread to.
 * @return CPU ID actually set on success, or `-1` if the operation failed or
 * the requested CPU ID was invalid.
 */
int setThreadCPU(pid_t pid, long cpuid);

/**
 * Sets a thread scheduling policy to real time with maximum priority.
 *
 * @param thread POSIX thread identifier to configure.
 * @return New scheduling priority on success, or `-1` if setting the
 * parameters failed.
 */
int setThreadRT(pthread_t thread);

/**
 * Manages the robot communication loop and shared low-level state.
 *
 * This class owns a `RobotApi` instance, runs the periodic receive/send loop on
 * a background thread, and exposes thread-safe access to the latest low-level
 * state and outgoing command buffer.
 */
class Communication final {
 public:
  /**
   * Creates a communication manager for the given robot API factory.
   *
   * @param robot_factory Registered `RobotApi` factory name.
   */
  explicit Communication(const std::string &robot_factory);
  ~Communication();

  const RobotApi::Ptr &api() const { return api_; }
  const RobotSpec &spec() const { return api_->getSpec(); }
  std::size_t dof() const { return dof_; }
  bool isCommunicating() const { return communicating_; }
  bool isConnected() const { return connected_; }
  bool isFrozen() const { return frozen_; }
  bool isActive() const { return active_; }
  std::size_t getFreq() const { return freq_; }
  std::size_t getTick() const { return tick_; }

  /**
   * Returns a snapshot of the latest low-level robot state.
   *
   * @return Copy of the most recently received low-level state.
   */
  LowState getLowState();

  /**
   * Replaces the pending low-level command buffer.
   *
   * @param low_cmd Next low-level command to send to the robot.
   */
  void setLowCmd(const LowCmd &low_cmd);

  /**
   * Blocks until the communication loop reaches at least the given tick.
   *
   * @param tick Minimum communication tick to wait for.
   */
  void waitUntil(std::size_t tick) const;

  /**
   * Enables or disables active command output.
   *
   * When disabled, the communication loop falls back to the configured damped
   * mode behavior.
   *
   * @param enabled Whether active command output should be enabled.
   */
  void setActive(bool enabled = true) { active_ = enabled; }

  /**
   * Marks the robot as frozen or unfrozen.
   *
   * A frozen communication loop forces inactive output handling.
   *
   * @param enabled Whether the robot should be treated as frozen.
   */
  void setFrozen(bool enabled = true) { frozen_ = enabled; }

  /**
   * Starts the background communication thread.
   *
   * If a communication thread is already running, it is stopped and restarted.
   */
  void launch();

  /**
   * Stops the background communication thread and disconnects from the robot.
   */
  void shutdown();

 private:
  /** Runs the periodic communication loop on the worker thread. */
  void mainLoop();
  /** Executes one receive/update/send communication cycle. */
  void mainEvent();

  RobotApi::Ptr api_;
  std::thread main_loop_thread_;
  std::atomic<pid_t> thread_id_{-1};

  std::atomic<bool> communicating_{false};
  std::atomic<bool> connected_{false};
  std::atomic<bool> active_{false};
  std::atomic<bool> frozen_{false};

  std::size_t dof_;
  std::size_t freq_;
  std::atomic<std::size_t> tick_{0};
  std::mutex mutex_;
  LowState low_state_;
  LowCmd low_cmd_;
};
}  // namespace stepit

#endif  // STEPIT_COMMUNICATION_H_
