#ifndef STEPIT_NEURO_POLICY_FIELD_OPS_H_
#define STEPIT_NEURO_POLICY_FIELD_OPS_H_

#include <stepit/field/operator.h>
#include <stepit/policy_neuro/module.h>

namespace stepit {
namespace neuro_policy {
class FieldOps : public Module {
 public:
  FieldOps(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);
  void init() override;
  bool reset() override;
  bool update(const LowState &, ControlRequests &, FieldMap &context) override;
  void postStep(const FieldMap &context) override;

 private:
  std::vector<field::Operator::Ptr> operations_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_FIELD_OPS_H_
