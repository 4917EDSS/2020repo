/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <frc/Solenoid.h>

class ControlPanelSub : public frc2::SubsystemBase {
 public:
  ControlPanelSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  void togglePosition(bool position);
  void setWheelSpeed(double speed);
 private:
   rev::CANSparkMax m_controlPanelMotor;
   frc::Solenoid m_controlPanelFlipper;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
