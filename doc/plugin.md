# Plugin Mechanism

StepIt employs a modular plugin architecture to extend functionalities like robot backends, control inputs, and policies.

## 1. Directory Structure

A typical plugin directory layout:

```text
my_plugin/
├── stepit_plugin_manifest.cmake
├── CMakeLists.txt
├── include/
│   └── stepit/
│       └── my_plugin/
│           └── my_plugin.h
└── src/
    └── my_plugin.cpp
```

## 2. Dependencies & Build (`CMakeLists.txt`)

StepIt discovers plugins by the presence of `stepit_plugin_manifest.cmake`.
Each manifest must declare the plugin name and plugin dependencies with `stepit_declare_plugin(...)`.

Use plain CMake logic in the manifest:

```cmake
stepit_declare_plugin(NAME my_plugin DEPENDS field_base)
find_package(Python3 QUIET COMPONENTS Interpreter Development)
if (NOT Python3_FOUND)
  stepit_plugin_mark_unbuildable("Missing Python3 interpreter or development files.")
  return()
endif ()
```

Then use the `stepit_add_plugin` macro in `CMakeLists.txt` to define the target.

```cmake
cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_example)

stepit_add_plugin(stepit_plugin_example
    SOURCES src/my_plugin.cpp
    INCLUDES include
    LINK_LIBS stepit_core
)
```

> **Note**: The plugin name must start with `stepit_plugin_`.

## 3. Implementing Interfaces

Inherit from the base classes defined in `stepit` headers and implement the required virtual methods.

```cpp
// my_plugin.h
#include <stepit/robot.h>

namespace stepit {
namespace my_plugin {
class MyRobot : public RobotApi {
 public:
  explicit MyRobot() : RobotApi("my_robot") {}
  void getControl(bool enable) override {}
  void setSend(const LowCmd &) override {}
  void getRecv(LowState &) override {}
  void send() override {}
  void recv() override {}
};
} // namespace my_plugin
} // namespace stepit
```


## 4. Registration

Register your implementations using the provided macros in your `.cpp` file with `STEPIT_REGISTER_*`, for example:

```cpp
#include <stepit/my_plugin/robot.h>

namespace stepit {
// Register custom robot API with default priority
STEPIT_REGISTER_ROBOTAPI(my_robot, kDefPriority, RobotApi::make<MyRobot>);
}  // namespace stepit
```

## 5. Lifecycle Hooks (Optional)

You can export C-style functions for initialization and cleanup:

- `extern "C" int stepit_plugin_init(int &argc, char *argv[])`: Called on load. Use to parse custom arguments.
- `extern "C" void stepit_plugin_cleanup()`: Called on unload.

## 6. Building & Loading

1.  Build your plugin using CMake.
2.  Ensure the compiled `.so` file is in a path searchable by StepIt. By default, it searches the following directories:
    - `<executable_dir>/`
    - `<executable_dir>/../lib/`
    - `<executable_dir>/../../lib/`
    - Other directories specified by the `STEPIT_EXTRA_PLUGIN_DIRS` environment variable.
3.  The `PluginManager` will automatically discover and load libraries matching `libstepit_plugin_*.so`.
