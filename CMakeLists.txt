cmake_minimum_required(VERSION 3.22)
project(HardwareInterface VERSION 1.0)
add_executable(
	HardwareInterface
	src/main.cpp
	src/controller/controller.cpp
	src/controller/device.cpp
	src/controller/sensor.cpp
	src/controller/sensors/lidar.cpp
	src/controller/sensors/BQ34Z100.cpp
	src/controller/actuator.cpp
	src/energy/energy.cpp
)
