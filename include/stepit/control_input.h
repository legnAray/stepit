#ifndef STEPIT_CONTROL_INPUT_H_
#define STEPIT_CONTROL_INPUT_H_

#include <cstdio>
#include <deque>
#include <future>
#include <list>
#include <memory>
#include <mutex>
#include <string>

#include <boost/optional.hpp>

#include <stepit/registry.h>

namespace stepit {
enum ErrorCode : std::uint8_t {
  kSuccess             = 0,
  kNotInCorrectState   = 1,
  kPolicyNotFound      = 2,
  kIncorrectArgument   = 3,
  kUnrecognizedRequest = 4,
  kInvalidRequest      = 255,
};

constexpr const char *kAgentNotInCorrectState = "Agent is not in correct state.";

struct ControlResponse {
  std::uint8_t status{kInvalidRequest};
  std::string message;
};

struct ControlRequest {
  explicit ControlRequest(const std::string &request_str = "");
  ControlRequest(const ControlRequest &other)                = delete;
  ControlRequest &operator=(const ControlRequest &other)     = delete;
  ControlRequest(ControlRequest &&other) noexcept            = default;
  ControlRequest &operator=(ControlRequest &&other) noexcept = default;
  const std::string &string() const { return string_; }
  const std::string &channel() const { return channel_; }
  const std::string &action() const { return action_; }
  const std::string &argument() const { return argument_; }

  template <typename... Args>
  bool parseArgument(const char *format, Args &...args) const {
    int result = std::sscanf(argument_.c_str(), format, &args...);
    return result == sizeof...(Args);
  }

  std::future<ControlResponse> getResponse() { return response_.get_future(); }
  void response(std::uint8_t code = 0, std::string message = "");
  void response(ErrorCode code, std::string message = "");
  bool channelMatches(const std::string &prefix) const { return channel_.rfind(prefix, 0) == 0; }
  friend std::ostream &operator<<(std::ostream &os, const ControlRequest &request) { return os << request.string_; }

 private:
  std::string string_, command_, channel_, action_, argument_;
  std::promise<ControlResponse> response_;
};

struct ControlRequests : public std::list<ControlRequest> {
  using std::list<ControlRequest>::list;
  ControlRequests filterByChannel(const std::string &channel_prefix);
};

class ControlInput : public Interface<ControlInput> {
 public:
  virtual bool available() const = 0;
  virtual void poll()            = 0;
  boost::optional<ControlRequest> pop();

 protected:
  std::future<ControlResponse> put(std::string request_str);

 private:
  std::mutex mutex_;
  std::deque<ControlRequest> requests_;
};

class MultipleControlInputs {
 public:
  explicit MultipleControlInputs(const std::vector<std::string> &input_names);
  bool available() const;
  void poll();
  boost::optional<ControlRequest> pop();

 private:
  std::vector<ControlInput::Ptr> inputs_;
};

template <typename Action>
Action lookupAction(const std::string &action, const std::map<std::string, Action> &action_map) {
  auto it = action_map.find(action);
  return it != action_map.end() ? it->second : Action::kInvalid;
}
}  // namespace stepit

#if FMT_VERSION >= 90000
template <>
struct fmt::formatter<stepit::ControlRequest> : fmt::ostream_formatter {};
#endif

#define STEPIT_REGISTER_CTRLINPUT(name, priority, factory) \
  static ::stepit::ControlInput::Registration _ctrl_##name##_registration(#name, priority, factory)

#endif  // STEPIT_CONTROL_INPUT_H_
