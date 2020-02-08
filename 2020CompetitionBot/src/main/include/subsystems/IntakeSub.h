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
  void setFrontRollerIntakePower(double power);
  void setMagazineIntakePower(double power);
  bool getFrontIntakeSensor();
  bool getMagazineFullSensor();
  bool getPowerCellSensor1();
  bool getPowerCellSensor2();
  bool getPowerCellSensor3();
  bool getPowerCellSensor4();

  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

 private:
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_frontRollerIntakeMotor;
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_topIntakeMotor;
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_bottomIntakeMotor;
  frc::DigitalInput m_frontIntakeSensor;
  frc::DigitalInput m_PowerCellSensor1;
  frc::DigitalInput m_PowerCellSensor2;
  frc::DigitalInput m_PowerCellSensor3;
  frc::DigitalInput m_PowerCellSensor4;
  // frc::DigitalInput m_magazineFullSensor;


};
