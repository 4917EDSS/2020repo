/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>

class IntakeSub : public frc2::SubsystemBase {
 public:
  IntakeSub();
  // negative speed sends balls to shooter positive takes them to magazine 
  void setIntake(double speed);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
 private:

  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_frontIntakeMotor;
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_interiorIntakeMotor;
  frc::DigitalInput m_magazineFullSensor;
};
