#pragma once

#include "../controller/controller.h"

class Module {
public:
  virtual void poll(Controller &controller);
  virtual bool condition(Controller &controller);
};
