#ifndef STEPIT_NEURO_POLICY_OBS_HISTORY_H_
#define STEPIT_NEURO_POLICY_OBS_HISTORY_H_

#include <string>
#include <vector>

#include <stepit/utils.h>
#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class ObsHistory : public Module {
 public:
  ObsHistory(const PolicySpec &policy_spec, const std::string &home_dir);
  void initFieldProperties() override;
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;

 private:
  enum class Order { kOldToNew, kNewToOld };
  enum class Pad { kFirst, kZeros };
  static Order parseOrder(const std::string &value);
  static Pad parsePad(const std::string &value);
  static std::string readSourceField(const YAML::Node &node);

  struct HistorySpec {
    std::string source_name;
    std::string target_name;
    FieldId source_id{};
    FieldId target_id{};
    std::size_t frames{};
    std::size_t frame_size{};
    Order order{Order::kOldToNew};
    Pad pad{Pad::kFirst};
    bool primed{false};
    StaticQueue<ArrXf> history;
    ArrXf buffer;
  };

  YAML::Node config_;
  std::vector<HistorySpec> specs_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_OBS_HISTORY_H_
