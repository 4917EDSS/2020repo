/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ColorSub.h"
#include <frc/smartdashboard/SmartDashboard.h>

static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
static constexpr frc::Color kRedTarget = frc::Color(0.473, 0.369, 0.154);
static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);

ColorSub::ColorSub() {
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);
}

// This method will be called once per scheduler run
void ColorSub::Periodic() {
    frc::Color detectedColor = m_colorSensor.GetColor();

    std::string colorString;
    double confidence = 0.0;
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

    if (matchedColor == kBlueTarget) {
      colorString = "Blue";
    } else if (matchedColor == kRedTarget) {
      colorString = "Red";
    } else if (matchedColor == kGreenTarget) {
      colorString = "Green";
    } else if (matchedColor == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("Confidence", confidence);
    frc::SmartDashboard::PutString("Detected Color", colorString);
}
