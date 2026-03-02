#include <numeric>

#include <stepit/field/operator.h>

namespace stepit {
namespace field {
AffineOperator::AffineOperator(const YAML::Node &config) : node_(config) {
  if (yml::hasValue(config, "field")) {
    auto field_name = yml::readAs<std::string>(config, "field");
    source_id_      = registerRequirement(field_name);
    target_id_      = source_id_;
  } else {
    STEPIT_ASSERT(config["source"] and config["target"],
                  "Affine op must contain 'field' or both 'source' and 'target'.");
    auto source_name = yml::readAs<std::string>(config, "source");
    auto target_name = yml::readAs<std::string>(config, "target");
    source_id_       = registerRequirement(source_name);
    target_id_       = registerProvision(target_name, 0);
  }
  STEPIT_ASSERT(not(config["scale"] and config["std"]), "Cannot specify both 'scale' and 'std' in an affine op.");
  STEPIT_ASSERT(not(config["bias"] and config["mean"]), "Cannot specify both 'bias' and 'mean' in an affine op.");
}

void AffineOperator::init() {
  auto field_size = getFieldSize(source_id_);
  scale_          = ArrXf::Ones(field_size);
  bias_           = ArrXf::Zero(field_size);

  if (yml::hasValue(node_, "scale")) {
    yml::setTo(node_, "scale", scale_);
  } else if (yml::hasValue(node_, "std")) {
    ArrXf std{ArrXf::Ones(field_size)};
    yml::setTo(node_, "std", std);
    STEPIT_ASSERT((std > kEPS).all(), "'std' values of affine op must be positive.");
    scale_ = std.cwiseInverse();
  }

  if (yml::hasValue(node_, "bias")) {
    yml::setTo(node_, "bias", bias_);
  } else if (yml::hasValue(node_, "mean")) {
    ArrXf mean{ArrXf::Zero(field_size)};
    yml::setTo(node_, "mean", mean);
    bias_ = -mean.cwiseProduct(scale_);
  }

  if (target_id_ != source_id_) {
    setFieldSize(target_id_, field_size);
  }
}

bool AffineOperator::update(FieldMap &context) {
  auto transformed    = context.at(source_id_).cwiseProduct(scale_) + bias_;
  context[target_id_] = std::move(transformed);
  return true;
}

ConcatOperator::ConcatOperator(const YAML::Node &config) {
  STEPIT_ASSERT(config["target"] and config["sources"], "Concat op must contain 'target' and 'sources'.");
  STEPIT_ASSERT(config["sources"].IsSequence(), "'sources' in concat op must be a sequence.");

  for (const auto &source_node : config["sources"]) {
    source_ids_.push_back(registerRequirement(yml::readAs<std::string>(source_node)));
  }
  target_id_ = registerProvision(yml::readAs<std::string>(config["target"]), 0);
}

void ConcatOperator::init() {
  FieldSize total_size = 0;
  for (auto source_id : source_ids_) {
    total_size += getFieldSize(source_id);
  }
  setFieldSize(target_id_, total_size);
  buffer_.resize(total_size);
}

bool ConcatOperator::update(FieldMap &context) {
  concatFields(context, source_ids_, buffer_);
  context[target_id_] = buffer_;
  return true;
}

ConstOperator::ConstOperator(const YAML::Node &config) {
  STEPIT_ASSERT(config["target"] or config["field"], "Const op must contain 'target' or 'field'.");
  STEPIT_ASSERT(yml::hasValue(config, "value"), "Const op must contain 'value'.");

  auto target_name = yml::hasValue(config, "target") ? yml::readAs<std::string>(config, "target")
                                                     : yml::readAs<std::string>(config, "field");

  auto value_node = config["value"];
  STEPIT_ASSERT(value_node.IsScalar() or value_node.IsSequence(), "Const op 'value' must be a scalar or sequence.");

  FieldSize size{};
  if (value_node.IsScalar()) {
    yml::setTo(config, "size", size);
    value_.setZero(size);
    yml::setTo(value_node, value_);
  } else {
    yml::setTo(value_node, value_);
    yml::setIf(config, "size", size);
    if (size > 0) {
      STEPIT_ASSERT(value_.size() == size, "Const op has mismatched 'size' and 'value' lengths.");
    } else {
      size = static_cast<FieldSize>(value_.size());
    }
  }

  target_id_ = registerProvision(target_name, size);
}

bool ConstOperator::update(FieldMap &context) {
  context[target_id_] = value_;
  return true;
}

CopyOperator::CopyOperator(const YAML::Node &config) {
  STEPIT_ASSERT(config["source"] and config["target"], "Copy op must contain 'source' and 'target'.");
  auto source_name = yml::readAs<std::string>(config, "source");
  auto target_name = yml::readAs<std::string>(config, "target");
  STEPIT_ASSERT(source_name != target_name, "Source and target cannot be the same in a copy op.");
  source_id_ = registerRequirement(source_name);
  target_id_ = registerProvision(target_name, 0);
}

void CopyOperator::init() { setFieldSize(target_id_, getFieldSize(source_id_)); }

bool CopyOperator::update(FieldMap &context) {
  context[target_id_] = context.at(source_id_);
  return true;
}

HistoryOperator::HistoryOperator(const YAML::Node &config) {
  STEPIT_ASSERT(config["source"] and config["target"], "Stack op must contain 'source' and 'target'.");
  source_id_ = registerRequirement(yml::readAs<std::string>(config, "source"));
  target_id_ = registerProvision(yml::readAs<std::string>(config, "target"), 0);

  yml::setTo(config["history_len"], history_len_);
  STEPIT_ASSERT(history_len_ > 0, "'history_len' of stack op must be greater than 0.");
  yml::setIf(config["newest_first"], newest_first_);
  yml::setIf(config["include_current_frame"], include_current_frame_);
  if (yml::hasValue(config, "default_value")) {
    yml::setTo(config, "default_value", default_value_);
  }
}

void HistoryOperator::init() {
  source_size_                = getFieldSize(source_id_);
  const FieldSize target_size = source_size_ * history_len_;
  setFieldSize(target_id_, target_size);

  if (default_value_.size() == 1) {
    default_value_ = VecXf::Constant(source_size_, default_value_[0]);
  } else if (default_value_.size() != 0) {
    STEPIT_ASSERT(default_value_.size() == source_size_,
                  "Default value size of stack op does not match source field size.");
  }

  history_.allocate(history_len_);
  output_.resize(target_size);
}

bool HistoryOperator::reset() {
  history_.clear();
  return true;
}

void HistoryOperator::push(const ArrXf &frame) {
  if (newest_first_) {
    history_.push_front(frame);
  } else {
    history_.push_back(frame);
  }
}

void HistoryOperator::updateOutput() {
  FieldSize offset = 0;
  for (const auto &frame : history_) {
    stackField(frame, offset, output_);
  }
  STEPIT_ASSERT(offset == output_.size(), "Stack op output size does not match total history size.");
}

bool HistoryOperator::update(FieldMap &context) {
  const auto &frame = context.at(source_id_);
  if (history_.empty()) {
    history_.fill(default_value_.size() > 0 ? default_value_ : frame);
  }

  if (include_current_frame_) {
    push(frame);
    updateOutput();
  } else {
    updateOutput();
    push(frame);
  }

  context[target_id_] = output_;
  return true;
}

MaskedFillOperator::MaskedFillOperator(const YAML::Node &config) {
  if (yml::hasValue(config, "field")) {
    auto field_name = yml::readAs<std::string>(config, "field");
    source_id_      = registerRequirement(field_name);
    target_id_      = source_id_;
  } else {
    STEPIT_ASSERT(config["source"] and config["target"],
                  "masked_fill op must contain 'field' or both 'source' and 'target'.");
    auto source_name = yml::readAs<std::string>(config, "source");
    auto target_name = yml::readAs<std::string>(config, "target");
    source_id_       = registerRequirement(source_name);
    target_id_       = registerProvision(target_name, 0);
  }

  if (yml::hasValue(config, "indices")) {
    const auto indices_node = config["indices"];
    STEPIT_ASSERT(indices_node.IsSequence() and indices_node.size() > 0,
                  "'indices' in masked_fill op must be a non-empty sequence.");
    for (const auto &index_node : indices_node) {
      indices_.push_back(yml::readAs<FieldSize>(index_node));
    }
  } else {
    auto start = yml::readAs<FieldSize>(config, "start");
    auto end   = yml::readAs<FieldSize>(config, "end");
    STEPIT_ASSERT(end > start, "Slice range [start={}, end={}) is invalid.", start, end);
    for (FieldSize i{start}; i < end; ++i) indices_.push_back(i);
  }

  yml::setIf(config, "value", value_);
}

void MaskedFillOperator::init() {
  auto source_size = getFieldSize(source_id_);
  for (auto index : indices_) {
    STEPIT_ASSERT(index < source_size, "masked_fill index {} is out of range [0, {}) for '{}'.", index, source_size,
                  getFieldName(source_id_));
  }
  setFieldSize(target_id_, source_size);
  buffer_.resize(source_size);
}

bool MaskedFillOperator::update(FieldMap &context) {
  buffer_ = context.at(source_id_);
  for (auto index : indices_) {
    buffer_[index] = value_;
  }
  context[target_id_] = buffer_;
  return true;
}

SliceOperator::SliceOperator(const YAML::Node &config) {
  STEPIT_ASSERT(config["source"] and config["target"], "Slice op must contain 'source' and 'target'.");
  source_id_ = registerRequirement(yml::readAs<std::string>(config, "source"));
  target_id_ = registerProvision(yml::readAs<std::string>(config, "target"), 0);

  if (yml::hasValue(config, "indices")) {
    const auto indices_node = config["indices"];
    STEPIT_ASSERT(indices_node.IsSequence() and indices_node.size() > 0,
                  "'indices' in slice op must be a non-empty sequence.");
    for (const auto &index_node : indices_node) {
      indices_.push_back(yml::readAs<FieldSize>(index_node));
    }
  } else {
    auto start = yml::readAs<FieldSize>(config, "start");
    auto end   = yml::readAs<FieldSize>(config, "end");
    STEPIT_ASSERT(end > start, "Slice range [start={}, end={}) is invalid.", start, end);
    for (FieldSize i{start}; i < end; ++i) indices_.push_back(i);
  }
}

void SliceOperator::init() {
  auto source_size = getFieldSize(source_id_);
  for (auto index : indices_) {
    STEPIT_ASSERT(index < source_size, "Slice index {} is out of range [0, {}) for '{}'.", index, source_size,
                  getFieldName(source_id_));
  }
  auto target_size = static_cast<FieldSize>(indices_.size());
  setFieldSize(target_id_, target_size);
  buffer_.resize(target_size);
}

bool SliceOperator::update(FieldMap &context) {
  const auto &source = context.at(source_id_);
  for (std::size_t i{}; i < indices_.size(); ++i) {
    buffer_[static_cast<Eigen::Index>(i)] = source[indices_[i]];
  }
  context[target_id_] = buffer_;
  return true;
}

SplitOperator::SplitOperator(const YAML::Node &config) {
  STEPIT_ASSERT(config["source"] and config["targets"], "Split op must contain 'source' and 'targets'.");
  source_id_ = registerRequirement(yml::readAs<std::string>(config, "source"));

  const auto targets_node = config["targets"];
  STEPIT_ASSERT(targets_node.IsSequence(), "'targets' in split op must be a sequence.");
  for (const auto &target_node : targets_node) {
    STEPIT_ASSERT(target_node.IsMap() and target_node["name"] and target_node["size"],
                  "Each split target must be a map containing keys 'name' and 'size'.");
    auto name = yml::readAs<std::string>(target_node, "name");
    auto size = yml::readAs<FieldSize>(target_node, "size");
    target_ids_.push_back(registerProvision(name, size));
    segment_sizes_.push_back(size);
  }
}

void SplitOperator::init() {
  auto source_size     = getFieldSize(source_id_);
  FieldSize total_size = std::accumulate(segment_sizes_.begin(), segment_sizes_.end(), static_cast<FieldSize>(0));
  STEPIT_ASSERT(total_size == source_size, "Split sizes ({}) do not match source size ({}) for '{}'.", total_size,
                source_size, getFieldName(source_id_));
}

bool SplitOperator::update(FieldMap &context) {
  splitFields(context.at(source_id_), target_ids_, context);
  return true;
}

STEPIT_REGISTER_FIELD_OPERATOR(affine, kDefPriority, Operator::make<AffineOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(concat, kDefPriority, Operator::make<ConcatOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(const, kDefPriority, Operator::make<ConstOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(copy, kDefPriority, Operator::make<CopyOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(masked_fill, kDefPriority, Operator::make<MaskedFillOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(slice, kDefPriority, Operator::make<SliceOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(history, kDefPriority, Operator::make<HistoryOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(split, kDefPriority, Operator::make<SplitOperator>);
}  // namespace field
}  // namespace stepit
