/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/ControlPanelSub.h"
#include "Constants.h"

ControlPanelSub::ControlPanelSub() 
  : m_controlPanelMotor(CanIds::kControlPanelMotor),
    m_controlPanelFlipper(PneumaticIds::kControlPanelFlipper){

  m_colourMatcher.AddColorMatch(ControlPanelConstants::kBlueTarget);
  m_colourMatcher.AddColorMatch(ControlPanelConstants::kGreenTarget);
  m_colourMatcher.AddColorMatch(ControlPanelConstants::kRedTarget);
  m_colourMatcher.AddColorMatch(ControlPanelConstants::kYellowTarget);
}

void ControlPanelSub::init() {
  flipArmUp(false);
}

void ControlPanelSub::Periodic() {}

bool ControlPanelSub::getArmPosition() {
  return m_controlPanelFlipper.Get();
}
void ControlPanelSub::flipArmUp(bool position){
  m_controlPanelFlipper.Set(position);
}

void ControlPanelSub::setWheelPower(double speed){
  m_controlPanelMotor.Set(speed);
}

frc::Color ControlPanelSub::getColour(){
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

  frc::SmartDashboard::PutNumber("Red", detectedColour.red);
  frc::SmartDashboard::PutNumber("Green", detectedColour.green);
  frc::SmartDashboard::PutNumber("Blue", detectedColour.blue);
  frc::SmartDashboard::PutNumber("Confidence", confidence);
  frc::SmartDashboard::PutString("Detected Colour", colourString);

  return matchedColour;
}

// TODO:  Move the colour conversion function from TrunControlPanelToColourCmd to here
// Robot  | Field
// -------|------
// Red    | Blue
// Yellow | Green
// Blue   | Red
// Green  | Yellow


