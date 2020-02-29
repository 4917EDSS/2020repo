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
#include <frc/Encoder.h>
#include <frc/AnalogInput.h>

constexpr double ktargetDistance = 7;

class IntakeSub : public frc2::SubsystemBase {
 public:
  IntakeSub();
  // negative speed sends balls to shooter positive takes them to magazine 
  void init();
  void setFrontRollerIntakePower(double power);
  void setMagazineIntakePower(double power);
  bool getFrontIntakeSensor();
  bool getMagazineFullSensor();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

 private:
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_frontRollerIntakeMotor;
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_topIntakeMotor;
  frc::DigitalInput m_frontIntakeSensorL;
  frc::DigitalInput m_frontIntakeSensorR;
  frc::DigitalInput m_magazineFullSensor;
};
