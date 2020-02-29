// 4917 proportional controller
// Will output max power until the delta between current and target location gets to control-start-delta
// Once at control-start-delta, output power will be proportional to how close current is to target
#pragma once


class ProportionalController {
 public:
  ProportionalController(double targetLocation, double controlStartDelta, double tolerance, double minPower, double maxPower);
  double getPower(double curLocation);

 private:
  double m_targetLocation;      // Where we are trying to get to (any unit)
  double m_controlStartDelta;   // Delta (target - current location) above this value will output max power (same units as current and target)
  double m_tolerance;           // Point above or below target where the motors will turn off (degrees or encoder ticks)
  double m_minPower;            // Power at which the mechanism moves slightly (0.0 to 1.0)
  double m_maxPower;            // Maximum absolute power that you want the mechanism to use (0.0 to 1.0)  
  
};
