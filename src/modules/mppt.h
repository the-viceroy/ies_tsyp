#ifndef MPPT_H
#define MPPT_H

#include "module.h"

/**
 * Maximum Power Point Tracking (MPPT) Module
 * 
 * This module implements the Incremental Conductance algorithm to maximize
 * power output from solar panels by dynamically adjusting the DC-DC converter
 * duty cycle.
 * 
 * How it works:
 * 1. Reads voltage and current from a sensor (e.g., INA219)
 * 2. Calculates the optimal operating point using incremental conductance:
 *    - At MPP: dI/dV = -I/V
 *    - Left of MPP: dI/dV > -I/V (increase duty to increase voltage)
 *    - Right of MPP: dI/dV < -I/V (decrease duty to decrease voltage)
 * 3. Adjusts PWM duty cycle to move toward maximum power point
 * 4. Tracks voltage/current changes to continuously optimize
 * 
 * Typical application: Solar panel power optimization in satellite power systems
 */
class MPPTModule : public Module {
public:
    /**
     * Constructor - Create an MPPT controller module
     * 
     * @param sensorId   ID of the voltage/current sensor (e.g., INA219) in Controller
     * @param actuatorId ID of the PWM actuator controlling the DC-DC converter
     * @param step       MPPT algorithm step size (0.0-1.0). Smaller = slower but more precise
     *                   Default: 0.01 (1% change per iteration)
     * @param dmin       Minimum allowed duty cycle (0.0-1.0)
     *                   Default: 0.05 (5% - prevents complete shutdown)
     * @param dmax       Maximum allowed duty cycle (0.0-1.0)
     *                   Default: 0.95 (95% - prevents full saturation)
     * 
     * Example: MPPTModule mppt(0, 1, 0.01, 0.05, 0.95);
     */
    MPPTModule(uint32_t sensorId, uint32_t actuatorId, 
               double step = 0.01, double dmin = 0.05, double dmax = 0.95);

    /**
     * Check if MPPT should run (Module interface)
     * 
     * @param controller Reference to the system controller
     * @return Always returns true - MPPT runs continuously
     */
    bool condition(Controller &controller) override;

    /**
     * Execute one MPPT iteration (Module interface)
     * 
     * Called periodically by the Manager. Each call:
     * 1. Reads voltage/current from the sensor
     * 2. Runs the incremental conductance algorithm
     * 3. Updates the duty cycle
     * 4. (Future) Commands the actuator with new duty cycle
     * 
     * @param controller Reference to the system controller for sensor/actuator access
     */
    void poll(Controller &controller) override;

    /**
     * Get the current PWM duty cycle
     * 
     * @return Duty cycle value from 0.0 to 1.0 (0% to 100%)
     */
    double getDuty() const;

    /**
     * Get the last calculated power output
     * 
     * @return Power in watts (W) from last measurement (V × I)
     */
    double getPower() const;

private:
    // Configuration
    uint32_t _sensorId;    // Which sensor to read V/I from
    uint32_t _actuatorId;  // Which actuator controls the converter
    
    // MPPT algorithm state variables
    double prevV;          // Previous voltage measurement (V) - for calculating dV
    double prevI;          // Previous current measurement (A) - for calculating dI
    double duty;           // Current PWM duty cycle (0.0 to 1.0)
    double stepSize;       // How much to change duty cycle each iteration
    double dutyMin;        // Lower limit for duty cycle
    double dutyMax;        // Upper limit for duty cycle
    double lastPower;      // Most recent power calculation (W)
    
    /**
     * Core MPPT algorithm implementation
     * Uses incremental conductance method to find maximum power point
     * 
     * @param voltage Current voltage measurement in volts
     * @param current Current measurement in amps
     */
    void updateMPPT(double voltage, double current);
};

#endif
