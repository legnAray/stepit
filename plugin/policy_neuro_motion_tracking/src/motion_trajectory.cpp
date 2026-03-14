#include <cstdint>

#include <stepit/policy_neuro/motion_trajectory.h>

namespace stepit {
namespace neuro_policy {
namespace {
std::string resolveMotionPath(const std::string &path, const std::string &home_dir) {
  STEPIT_ASSERT(not path.empty(), "'path' cannot be empty.");
  return path[0] == '/' ? path : joinPaths(home_dir, path);
}
}  // namespace

MotionTrajectory::MotionTrajectory(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "motion_trajectory")),
      clip_(resolveMotionPath(config_["path"].as<std::string>(name_), policy_spec.home_dir),
            config_["dataloader_factory"].as<std::string>("")) {
  const auto control_freq = static_cast<double>(policy_spec.control_freq);
  STEPIT_ASSERT(control_freq > 0., "Control frequency should be positive, but got {}.", policy_spec.control_freq);
  const auto default_offsets = config_["offsets"].as<std::vector<std::int64_t>>({0});
  STEPIT_ASSERT(not default_offsets.empty(), "'offsets' cannot be empty.");

  const auto field_nodes = config_["field"];
  field_nodes.assertSequence();
  for (const auto &node : field_nodes) {
    initField(node, default_offsets, control_freq);
  }

  STEPIT_ASSERT(num_frames_ > 0, "Loaded trajectory '{}' with 0 frames.", clip_.path());
  STEPIT_DBUG("Loaded trajectory with {} frames and {} fields from '{}'.", num_frames_, fields_.size(), clip_.path());
}

void MotionTrajectory::initField(const yml::Node &node, const std::vector<std::int64_t> &default_offsets,
                                 double control_freq) {
  const auto name          = node["name"].as<std::string>();
  const auto key           = node["key"].as<std::string>();
  const auto type          = node["type"].as<std::string>("numeric");
  const auto differentiate = node["differentiate"].as<bool>(false);
  auto indices             = node.as<yml::Indices>();
  auto field_frames        = clip_.loadField(name, key, indices, type, differentiate, control_freq);

  FieldView field;
  node["offsets"].to(field.offsets, true);
  if (field.offsets.empty()) field.offsets = default_offsets;
  const auto field_num_frames = field_frames.size();
  if (num_frames_ == 0) {
    num_frames_ = field_num_frames;
  } else {
    STEPIT_ASSERT(field_num_frames == num_frames_,
                  "Field '{}' (key '{}') resolves to {} control frames, but previous fields resolved to {}.", name, key,
                  field_num_frames, num_frames_);
  }

  field.frame_size = field_frames.front().size();
  field.source     = std::move(field_frames);
  field.field_size = field.frame_size * field.offsets.size();
  if (node["size"].hasValue()) {
    const auto declared_size = node["size"].as<std::size_t>();
    STEPIT_ASSERT(declared_size == field.field_size,
                  "Field size specified ({}) does not match the stacked size ({}) of field '{}'.", declared_size,
                  field.field_size, name);
  }

  field.field_id = registerProvision(name, static_cast<FieldSize>(field.field_size));
  field.buffer.resize(static_cast<Eigen::Index>(field.field_size));
  fields_.push_back(std::move(field));
}

bool MotionTrajectory::reset() {
  frame_idx_ = 0;
  return true;
}

bool MotionTrajectory::update(const LowState &, ControlRequests &, FieldMap &context) {
  const auto frame_idx     = static_cast<std::int64_t>(frame_idx_);
  const auto max_frame_idx = static_cast<std::int64_t>(num_frames_ - 1);
  for (auto &field : fields_) {
    const auto slice_size = static_cast<Eigen::Index>(field.frame_size);
    auto &buffer          = field.buffer;

    Eigen::Index write_offset = 0;
    for (const auto frame_offset : field.offsets) {
      const auto sample_idx = static_cast<std::size_t>(clamp(frame_idx + frame_offset, std::int64_t{0}, max_frame_idx));
      buffer.segment(write_offset, slice_size) = field.source[sample_idx];
      write_offset += slice_size;
    }
    context[field.field_id] = field.buffer;
  }

  if (frame_idx_ + 1 < num_frames_) ++frame_idx_;
  return true;
}

STEPIT_REGISTER_MODULE(motion_trajectory, kDefPriority, Module::make<MotionTrajectory>);
}  // namespace neuro_policy
}  // namespace stepit
