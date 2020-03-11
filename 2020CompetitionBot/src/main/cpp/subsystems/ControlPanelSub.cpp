/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "subsystems/ControlPanelSub.h"
#include "Constants.h"

ControlPanelSub::ControlPanelSub() 
  : m_controlPanelMotor(CanIds::kControlPanelMotor),
    m_controlPanelFlipper(PneumaticIds::kControlPanelFlipper) {

  m_colourMatcher.AddColorMatch(ControlPanelConstants::kBlueTarget);
  m_colourMatcher.AddColorMatch(ControlPanelConstants::kGreenTarget);
  m_colourMatcher.AddColorMatch(ControlPanelConstants::kRedTarget);
  m_colourMatcher.AddColorMatch(ControlPanelConstants::kYellowTarget);

  frc::SmartDashboard::PutString("Field Colour", "Unkown");
}

void ControlPanelSub::init() {
  flipArmUp(true);
}

void ControlPanelSub::Periodic() {
  static int periodicCounts = 0;

  // Don't check every loop
  if(++periodicCounts >= 25) {
    determineColourToTurnTo();
    periodicCounts = 0;
  }
  
}

bool ControlPanelSub::isArmUp() {
  return !m_controlPanelFlipper.Get();
}

void ControlPanelSub::flipArmUp(bool position) {
  m_controlPanelFlipper.Set(!position);
}

void ControlPanelSub::setWheelPower(double power) {
  m_controlPanelMotor.Set(power);
}

frc::Color ControlPanelSub::getColour() {
  frc::Color detectedColour = m_colourSensor.GetColor();

  std::string colourString;
  double confidence = 0.0;
  frc::Color matchedColour = m_colourMatcher.MatchClosestColor(detectedColour, confidence);

  if (matchedColour == ControlPanelConstants::kBlueTarget) {
    colourString = "Blue";
  } 
  else if (matchedColour == ControlPanelConstants::kRedTarget) {
    colourString = "Red";
  } 
  else if (matchedColour == ControlPanelConstants::kGreenTarget) {
    colourString = "Green";
  } 
  else if (matchedColour == ControlPanelConstants::kYellowTarget) {
    colourString = "Yellow";
  } 
  else {
    colourString = "Unknown";
  }

  // frc::SmartDashboard::PutNumber("Red", detectedColour.red);
  // frc::SmartDashboard::PutNumber("Green", detectedColour.green);
  // frc::SmartDashboard::PutNumber("Blue", detectedColour.blue);
  // frc::SmartDashboard::PutNumber("Confidence", confidence);
  frc::SmartDashboard::PutString("Detected Colour", colourString);
  return matchedColour;
}

// Convert what field tells us to what the robot turns to.
// Robot  | Field
// -------|------
// Red    | Blue
// Yellow | Green
// Blue   | Red
// Green  | Yellow
void ControlPanelSub::determineColourToTurnTo() {
  bool colourToTurnToValid = true;  // Assume true until we find out it's not valid

  // Get colour from the field
  std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  frc::SmartDashboard::PutString("Field Colour", gameData);

  if(gameData.length() > 0) {
    switch (gameData[0]) {
      case 'G' :
        //Green case code
        m_colourToTurnTo = ControlPanelConstants::kYellowTarget;
        break;
      case 'B' :
        //Blue case code
        m_colourToTurnTo = ControlPanelConstants::kRedTarget;
        break;
      case 'Y' :
        //Yellow case code
        m_colourToTurnTo = ControlPanelConstants::kGreenTarget;
        break;
      case 'R' :
        //Red case code
        m_colourToTurnTo = ControlPanelConstants::kBlueTarget;
        break;
      default :
        colourToTurnToValid = false;
        break;
    }
  } 
  else {
    colourToTurnToValid = false;
  }

  m_colourToTurnToValid = colourToTurnToValid;
  frc::SmartDashboard::PutBoolean("Field Colour OK", m_colourToTurnToValid);
}

frc::Color ControlPanelSub::getColourToTurnTo() {
  return m_colourToTurnTo;
}

bool ControlPanelSub::isColourToTurnToValid() {
  return m_colourToTurnToValid;
}
