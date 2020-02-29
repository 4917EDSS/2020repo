#include "components/ProportionalController.h"

ProportionalController::ProportionalController(double targetLocation, double controlStartDelta, double minPower, double maxPower) 
  : m_targetLocation(targetLocation), m_controlStartDelta(controlStartDelta), m_minPower(minPower), m_maxPower(maxPower) {

}

double ProportionalController::getPower(double curLocation) {
  double outputPower = 0.0;

  // Determine how far we are from the target and in which direction we want to go
  bool forward = true;
  double delta = m_targetLocation - curLocation;
  if(delta < 0.0) {
    forward = false;
    delta *= -1.0;  // Make sure delta is positive for calcs
  }

  // If the delta is bigger than the point at which we want to apply proportional control, use max power
  if(delta > m_controlStartDelta) {
    outputPower = m_maxPower;
  }
  // Otherwise, apply power proportionaly to how close we are to the target
  else {
    double targetClosenessFraction = delta / m_controlStartDelta;
    double controlPowerRange = m_maxPower - m_minPower;
    outputPower = (targetClosenessFraction * controlPowerRange) + m_minPower;
  }

  // Set the direction of the output power
  if(!forward) {
    outputPower *= -1.0;
  }

  return outputPower;
}


// Test code
  // ProportionalController pc(100, 50, 0.2, 0.5);
  // std::cout << "\n0.0 to 100.0\n";
  // for(double i = 0.0; i < 100.0; i+=1.0) {
  //   std::cout << i << ", " << pc.getPower(i) << std::endl;
  // }
  // std::cout << "\n200.0 to 100.0\n";
  // for(double i = 200.0; i > 100.0; i-=1.0) {
  //   std::cout << i << ", " << pc.getPower(i) << std::endl;
  // }

  // ProportionalController pc2(-100, 50, 0.2, 0.5);
  // std::cout << "\n0.0 to -100.0\n";
  // for(double i = 0.0; i > -100.0; i-=1.0) {
  //   std::cout << i << ", " << pc2.getPower(i) << std::endl;
  // }
  // std::cout << "\n-200.0 to -100.0\n";
  // for(double i = -200.0; i < -100.0; i+=1.0) {
  //   std::cout << i << ", " << pc2.getPower(i) << std::endl;
  // }