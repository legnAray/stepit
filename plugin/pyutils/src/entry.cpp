#include <stepit/pyutils/python_runtime.h>

extern "C" {
int stepit_plugin_init(int &argc, char **argv) {
  stepit::ensurePythonInterpreter();
  return 0;
}

int stepit_plugin_cleanup(int &argc, char **argv) {
  stepit::releasePythonInterpreter();
  return 0;
}
}
