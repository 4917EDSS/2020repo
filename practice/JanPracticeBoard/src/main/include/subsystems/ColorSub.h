/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/util/color.h>


class ColorSub : public frc2::SubsystemBase {
 public:

  ColorSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

 private:
   static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
    rev::ColorSensorV3 m_colorSensor{i2cPort};
    rev::ColorMatch m_colorMatcher;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
