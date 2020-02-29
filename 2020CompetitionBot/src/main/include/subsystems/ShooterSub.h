/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>

class ShooterSub : public frc2::SubsystemBase {
 public:
  ShooterSub();
  void init();
  void Periodic() override;
  void setSpeed(double speed);
  void setFeedSpeed(double feedSpeed);
  void setHoodSpeed(double hoodSpeed);
  double getHoodEncoder();
  int getSpeed();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX m_shooterMotor1;
  WPI_TalonFX m_shooterMotor2;
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_feederMotor;
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_hoodMotor;
};
