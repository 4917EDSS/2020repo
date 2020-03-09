/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/util/color.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace ControlPanelConstants {
    constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
    constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
    constexpr frc::Color kRedTarget = frc::Color(0.473, 0.369, 0.154);
    constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
    constexpr double kMaxWheelSpeed = 1;
}

class ControlPanelSub : public frc2::SubsystemBase {
 public:
  ControlPanelSub();

  void init();
  void Periodic();
  bool isArmUp();
  void flipArmUp(bool position);
  void setWheelPower(double power);
  frc::Color getColour();

 private:
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  
  rev::ColorSensorV3 m_colourSensor{i2cPort};
  rev::ColorMatch m_colourMatcher;
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_controlPanelMotor;
  frc::Solenoid m_controlPanelFlipper;
};
