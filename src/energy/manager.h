#pragma once

#include "../controller/controller.h"
#include "../modules/module.h"
#include <functional>
#include <vector>

class Manager {
public:
  Manager(Controller controller, std::vector<Module> modules)
      : controller(std::move(controller)), modules(std::move(modules)) {}
  void poll();

private:
  Controller controller;
  std::vector<Module> modules;
};

class Monitor {
public:
  Monitor(Sensor &mSensor, std::vector<unsigned int> mDeviceId,
          uint8_t mPriority)
      : mSensor(mSensor), mDeviceId(std::move(mDeviceId)),
        mPriority(mPriority) {}
  bool evaluate(Controller &controller);

private:
  Sensor &mSensor;
  std::vector<unsigned int> mDeviceId;
  uint8_t mPriority;
};
