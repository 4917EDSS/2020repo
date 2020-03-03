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

class ControlPanelSub : public frc2::SubsystemBase {
 public:
  ControlPanelSub();

  void init();
  void Periodic();
  bool isArmUp();
  void flipArmUp(bool position);
  void setWheelPower(double speed);
  frc::Color getColour();

 private:
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  
  rev::ColorSensorV3 m_colourSensor{i2cPort};
  rev::ColorMatch m_colourMatcher;
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_controlPanelMotor;
  frc::Solenoid m_controlPanelFlipper;
};
