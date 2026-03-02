#include <stepit/policy_neuro/field_ops.h>

namespace stepit {
namespace neuro_policy {
FieldOps::FieldOps(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "field_ops")) {
  std::string ops_key = yml::getDefinedKey(config_, "ops", "operators");
  auto ops_node       = config_[ops_key];
  STEPIT_ASSERT(ops_node.IsSequence(), "'{}' must contain an 'operators' sequence.", config_filename_);

  for (const auto &op_config : ops_node) {
    STEPIT_ASSERT(op_config.IsMap(), "Each field op must be a map.");
    STEPIT_ASSERT(yml::hasValue(op_config, "type"), "Each field op must contain a 'type'.");
    auto op_type   = yml::readAs<std::string>(op_config["type"]);
    auto operation = field::Operator::make(op_type, op_config);

    for (auto field_id : operation->requirements()) {
      registerRequirement(field_id);
    }
    for (auto field_id : operation->provisions()) {
      registerProvision(field_id);
    }

    operations_.push_back(std::move(operation));
  }
}

void FieldOps::init() {
  for (auto &operation : operations_) {
    operation->init();
  }
}

bool FieldOps::reset() {
  for (auto &operation : operations_) {
    if (not operation->reset()) return false;
  }
  return true;
}

bool FieldOps::update(const LowState &, ControlRequests &, FieldMap &context) {
  for (auto &operation : operations_) {
    if (not operation->update(context)) return false;
  }
  return true;
}

STEPIT_REGISTER_MODULE(field_ops, kDefPriority, Module::make<FieldOps>);
}  // namespace neuro_policy
}  // namespace stepit
