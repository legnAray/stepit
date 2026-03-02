#ifndef STEPIT_FIELD_OPERATION_H_
#define STEPIT_FIELD_OPERATION_H_

#include <stepit/field/field.h>

namespace stepit {
namespace field {
class Operator : public Node, public Interface<Operator, const YAML::Node & /* config */> {
 public:
  virtual void init() {}
  virtual bool reset() { return true; }
  virtual bool update(FieldMap &context) = 0;
  virtual void finalize(const FieldMap &context) {}
};

class AffineOperator : public Operator {
 public:
  explicit AffineOperator(const YAML::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  YAML::Node node_;
  FieldId source_id_{};
  FieldId target_id_{};
  ArrXf scale_;
  ArrXf bias_;
};

class ConcatOperator : public Operator {
 public:
  explicit ConcatOperator(const YAML::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldIdVec source_ids_;
  FieldId target_id_{};
  ArrXf buffer_;
};

class ConstOperator : public Operator {
 public:
  explicit ConstOperator(const YAML::Node &config);

  bool update(FieldMap &context) override;

 private:
  FieldId target_id_{};
  ArrXf value_;
};

class CopyOperator : public Operator {
 public:
  explicit CopyOperator(const YAML::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldId target_id_{};
};

class HistoryOperator : public Operator {
 public:
  explicit HistoryOperator(const YAML::Node &config);

  void init() override;
  bool reset() override;
  bool update(FieldMap &context) override;

 private:
  void push(const ArrXf &frame);
  void updateOutput();

  FieldId source_id_{};
  FieldId target_id_{};
  std::uint32_t history_len_{};
  FieldSize source_size_{};
  bool newest_first_{true};
  bool include_current_frame_{true};
  ArrXf default_value_;
  RingBuffer<ArrXf> history_;
  ArrXf output_;
};

class MaskedFillOperator : public Operator {
 public:
  explicit MaskedFillOperator(const YAML::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldId target_id_{};
  std::vector<FieldSize> indices_;
  float value_{};
  ArrXf buffer_;
};

class SliceOperator : public Operator {
 public:
  explicit SliceOperator(const YAML::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldId target_id_{};
  std::vector<FieldSize> indices_;
  ArrXf buffer_;
};

class SplitOperator : public Operator {
 public:
  explicit SplitOperator(const YAML::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldIdVec target_ids_;
  std::vector<FieldSize> segment_sizes_;
};
}  // namespace field
}  // namespace stepit

#define STEPIT_REGISTER_FIELD_OPERATOR(name, priority, factory) \
  static ::stepit::field::Operator::Registration _field_operation_##name##_registration(#name, priority, factory)

#endif  // STEPIT_FIELD_OPERATION_H_
