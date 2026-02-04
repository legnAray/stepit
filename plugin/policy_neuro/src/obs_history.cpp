#include <stepit/policy_neuro/obs_history.h>

namespace stepit {
namespace neuro_policy {
ObsHistory::Order ObsHistory::parseOrder(const std::string &value) {
  std::string order = value;
  toLowercaseInplace(order);
  if (order == "old_to_new" || order == "old2new" || order == "otn") return ObsHistory::Order::kOldToNew;
  if (order == "new_to_old" || order == "new2old" || order == "nto") return ObsHistory::Order::kNewToOld;
  STEPIT_ERROR("Unknown obs_history order '{}'.", value);
  return ObsHistory::Order::kOldToNew;
}

ObsHistory::Pad ObsHistory::parsePad(const std::string &value) {
  std::string pad = value;
  toLowercaseInplace(pad);
  if (pad == "first" || pad == "first_frame" || pad == "firstframe") return ObsHistory::Pad::kFirst;
  if (pad == "zero" || pad == "zeros" || pad == "zero_pad") return ObsHistory::Pad::kZeros;
  STEPIT_ERROR("Unknown obs_history pad '{}'.", value);
  return ObsHistory::Pad::kFirst;
}

std::string ObsHistory::readSourceField(const YAML::Node &node) {
  std::string source_field;
  if (yml::isValid(node, "source_field")) {
    yml::setTo(node["source_field"], source_field);
  } else if (yml::isValid(node, "source")) {
    yml::setTo(node["source"], source_field);
  } else {
    STEPIT_ERROR("Missing key 'source_field' for obs_history definition '{}'.", yml::formatNode(node));
  }
  return source_field;
}

ObsHistory::ObsHistory(const PolicySpec &, const std::string &home_dir)
    : config_(yml::loadFile(home_dir + "/obs_history.yml")) {
  STEPIT_ASSERT(config_.IsMap(), "'obs_history.yml' must contain a map of obs history configurations.");

  for (const auto &node : config_) {
    HistorySpec spec;
    yml::setTo(node.first, spec.target_name);
    STEPIT_ASSERT(node.second.IsMap(), "Definition for '{}' must be a map.", spec.target_name);

    spec.source_name = readSourceField(node.second);
    yml::setTo(node.second["frames"], spec.frames);
    STEPIT_ASSERT(spec.frames > 0, "'frames' must be greater than 0 for '{}'.", spec.target_name);

    std::string order_str = "old_to_new";
    if (yml::isValid(node.second, "order")) yml::setTo(node.second["order"], order_str);
    spec.order = parseOrder(order_str);

    std::string pad_str = "first";
    if (yml::isValid(node.second, "pad")) yml::setTo(node.second["pad"], pad_str);
    spec.pad = parsePad(pad_str);

    spec.source_id = registerRequirement(spec.source_name);
    spec.target_id = registerProvision(spec.target_name, 0);
    specs_.push_back(std::move(spec));
  }
}

void ObsHistory::initFieldProperties() {
  for (auto &spec : specs_) {
    spec.frame_size = getFieldSize(spec.source_id);
    STEPIT_ASSERT(spec.frame_size > 0, "Size of '{}' is undefined.", getFieldName(spec.source_id));

    spec.history.allocate(spec.frames);
    spec.buffer.resize(spec.frames * spec.frame_size);
    setFieldSize(spec.target_id, spec.frames * spec.frame_size);
  }
}

bool ObsHistory::reset() {
  for (auto &spec : specs_) {
    spec.history.clear();
    spec.primed = false;
  }
  return true;
}

bool ObsHistory::update(const LowState &, ControlRequests &, FieldMap &result) {
  for (auto &spec : specs_) {
    const auto &frame = result.at(spec.source_id);
    STEPIT_ASSERT_EQ(static_cast<std::size_t>(frame.size()), spec.frame_size, "Field '{}' size mismatch.",
                     getFieldName(spec.source_id));

    if (spec.pad == Pad::kFirst && !spec.primed) {
      for (std::size_t i{}; i < spec.frames; ++i) spec.history.push_back(frame);
      spec.primed = true;
    } else {
      spec.history.push_back(frame);
    }

    std::size_t history_size = spec.history.size();
    std::size_t missing = spec.frames > history_size ? spec.frames - history_size : 0;

    std::size_t index = 0;
    auto stack_frame = [&](const ArrXf &value) {
      spec.buffer.segment(index, value.size()) = value;
      index += static_cast<std::size_t>(value.size());
    };

    if (spec.order == Order::kOldToNew) {
      if (missing > 0 && spec.pad == Pad::kZeros) {
        ArrXf zero = ArrXf::Zero(spec.frame_size);
        for (std::size_t i{}; i < missing; ++i) stack_frame(zero);
      }
      for (std::size_t i{}; i < history_size; ++i) stack_frame(spec.history.at(static_cast<int64_t>(i)));
    } else {
      for (std::size_t i{}; i < history_size; ++i) {
        auto idx = static_cast<int64_t>(history_size - 1 - i);
        stack_frame(spec.history.at(idx));
      }
      if (missing > 0 && spec.pad == Pad::kZeros) {
        ArrXf zero = ArrXf::Zero(spec.frame_size);
        for (std::size_t i{}; i < missing; ++i) stack_frame(zero);
      }
    }

    result[spec.target_id] = spec.buffer;
  }
  return true;
}

STEPIT_REGISTER_MODULE(obs_history, kDefPriority, Module::make<ObsHistory>);
}  // namespace neuro_policy
}  // namespace stepit
