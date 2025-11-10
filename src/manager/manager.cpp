#include "manager.h"

void Manager::poll() {
  for (int i = 0; i < this->modules.size(); i++) {
    if (modules[i].condition(controller)) {
      modules[i].poll(controller);
    }
  }
}
