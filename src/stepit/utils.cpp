#include <stepit/utils.h>

namespace stepit {
namespace {
std::vector<std::string> buildDefaultConfigSearchPaths() {
  std::vector<std::string> config_dirs;
  getenv("STEPIT_CONFIG_DIR", config_dirs);
#ifdef STEPIT_CONFIG_DIR
  config_dirs.emplace_back(STEPIT_CONFIG_DIR);
#else
  const char *home_dir = std::getenv("HOME");
  config_dirs.emplace_back(joinPaths(home_dir != nullptr ? home_dir : ".", ".config", "stepit"));
#endif  // STEPIT_CONFIG_DIR
  return config_dirs;
}
}  // namespace

const std::vector<std::string> &getConfigSearchPaths() {
  static const std::vector<std::string> config_dirs = buildDefaultConfigSearchPaths();
  return config_dirs;
}

yml::Node loadGlobalConfigYaml(const std::string &relative_path) {
  const auto &config_dirs = getConfigSearchPaths();
  std::string yaml_path;
  for (const auto &config_dir : config_dirs) {
    yaml_path = relative_path.empty() ? config_dir : joinPaths(config_dir, relative_path);
    if (fs::exists(yaml_path)) break;
  }
  STEPIT_ASSERT(not yaml_path.empty(), "File '{}' not found in {}.", relative_path, config_dirs);
  return yml::loadFile(yaml_path);
}
}  // namespace stepit
