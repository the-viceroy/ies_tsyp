#include "controller/controller.h"
#include "manager/manager.h"
#include "modules/modules.h"

int main(int argc, char *argv[]) {
  Controller controller;
  Manager manager(controller, std::vector<Module>());

  while (true) {
    manager.poll();
  }
  return 0;
}
