#include "BQ34Z100.h"

#include <cinttypes>
#include <vector>
#include <cmath>

// Constructor: set I2C frequency
BQ34Z100::BQ34Z100(I2C& i2c, int hz)
  : _i2c(i2c) {
  _i2c.frequency(hz);
}



// Low-level multi-byte register read helper
uint32_t BQ34Z100::read(Command command, const uint8_t length) {
  uint32_t val = 0;
  for (int i = 0; i < length; i++) {
	uint8_t cmdByte = static_cast<uint8_t>(command) + i;
	int writeResult = _i2c.write(
		GAUGE_ADDRESS | 0x0,
		reinterpret_cast<char*>(&cmdByte),
		1);
	if (writeResult != 0) {
	  printf("I2C write error when setting register address\r\n");
	}

	char readByte = 0;
	int readResult = _i2c.read(
		GAUGE_ADDRESS | 0x1,
		&readByte,
		1);
	if (readResult != 0) {
	  printf("I2C read error when reading data\r\n");
	}

	val |= (static_cast<uint32_t>(static_cast<uint8_t>(readByte)) << (8 * i));
  }
  return val;
}

// getters
uint8_t BQ34Z100::getSOC() { return read(Command::StateOfCharge, 1); }

uint16_t BQ34Z100::getVoltage() { return read(Command::Voltage, 2); }

int16_t BQ34Z100::getCurrent() { return static_cast<int16_t>(read(Command::Current, 2)); }

double BQ34Z100::getTemperature() {
  // Temperature returned in 0.1 K; convert to Celsius
  return (read(Command::Temperature, 2) / 10.0) - 273.15;
}

// Sensor::read override: pack a compact telemetry frame
std::vector<uint8_t> BQ34Z100::read() {
  const uint16_t voltage = getVoltage();        // mV
  const int16_t current = getCurrent();         // mA (signed)
  const uint8_t soc = getSOC();                 // %
  const int16_t tempCenti = static_cast<int16_t>(std::lround(getTemperature() * 100.0)); // °C * 100

  std::vector<uint8_t> payload;
  // voltage (LE)
  payload.push_back(static_cast<uint8_t>(voltage & 0xFF));
  payload.push_back(static_cast<uint8_t>((voltage >> 8) & 0xFF));
  // current (LE)
  payload.push_back(static_cast<uint8_t>(current & 0xFF));
  payload.push_back(static_cast<uint8_t>((current >> 8) & 0xFF));
  // state of charge
  payload.push_back(soc);
  // temperature *100 (LE)
  payload.push_back(static_cast<uint8_t>(tempCenti & 0xFF));
  payload.push_back(static_cast<uint8_t>((tempCenti >> 8) & 0xFF));
  return payload;
}

