#ifndef STEPIT_PUBLISHER_H_
#define STEPIT_PUBLISHER_H_

#include <map>
#include <memory>
#include <string>

#include <stepit/registry.h>
#include <stepit/robot.h>

namespace stepit {
class Publisher: public Interface<Publisher> {
 public:
  static Publisher &instance();

  bool hasStatus(const std::string &name) const { return named_status_.find(name) != named_status_.end(); }
  void updateStatus(const std::string &name, const std::string &value) { named_status_[name] = value; }
  void removeStatus(const std::string &name) { named_status_.erase(name); }

  virtual void publishStatus() {}
  virtual void publishLowLevel(const RobotSpec &spec, const LowState &state, const LowCmd &cmd) {}
  virtual void publishArray(const std::string &name, cArrXf arr) {}

 protected:
  std::map<std::string, std::string> named_status_;
};

namespace publisher {
// Helper accessors

struct Filter {
  Filter();
  bool publish_status{true};
  bool publish_low_level{true};
  bool publish_array{true};
};
extern const Filter g_filter;
inline Publisher &publisher() { return Publisher::instance(); }
inline bool hasStatus(const std::string &name) { return publisher().hasStatus(name); }
inline void updateStatus(const std::string &name, const std::string &value) { publisher().updateStatus(name, value); }
inline void updateStatus(const std::string &name, const char *value) { publisher().updateStatus(name, value); }
template <typename T>
void updateStatus(const std::string &name, const T &value) {
  updateStatus(name, std::to_string(value));
}
inline void removeStatus(const std::string &name) { publisher().removeStatus(name); }

inline void publishStatus() {
  if (g_filter.publish_status) publisher().publishStatus();
}
inline void publishLowLevel(const RobotSpec &spec, const LowState &low_state, const LowCmd &low_cmd) {
  if (g_filter.publish_low_level) publisher().publishLowLevel(spec, low_state, low_cmd);
}
inline void publishArray(const std::string &name, cArrXf arr) {
  if (g_filter.publish_array) publisher().publishArray(name, arr);
}

class StatusRegistration {
 public:
  StatusRegistration(const std::string &name);
  ~StatusRegistration();
  StatusRegistration(const StatusRegistration &)            = delete;
  StatusRegistration &operator=(const StatusRegistration &) = delete;
  StatusRegistration(StatusRegistration &&other) noexcept;
  StatusRegistration &operator=(StatusRegistration &&other) noexcept;
  template <typename T>
  void update(const T &value) {
    updateStatus(name_, value);
  }

  using Ptr = std::unique_ptr<StatusRegistration>;
  static Ptr make(std::string name) { return std::make_unique<StatusRegistration>(std::move(name)); }

 private:
  std::string name_;
};
}  // namespace publisher
}  // namespace stepit

#define STEPIT_REGISTER_PUBLISHER(name, priority, factory) \
  static ::stepit::Publisher::Registration _publisher_##name##_registration(#name, priority, factory)

#endif  // STEPIT_PUBLISHER_H_
