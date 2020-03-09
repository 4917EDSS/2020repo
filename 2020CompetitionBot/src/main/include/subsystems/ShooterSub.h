/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>

namespace ShooterConstants {
    constexpr double kHighHood = 0.0;
    constexpr double kLowHood = 15500;
    constexpr double kMaxRPM = 21750;
    constexpr double kCloseTargetSpeed = 15000;
    constexpr double kFarTargetSpeed = 17800;   
}

class ShooterSub : public frc2::SubsystemBase {
 public:
  ShooterSub();
  void init();
  void Periodic() override;
  void setPower(double speed);
  void setFeedSpeed(double feedSpeed);
  int getSpeed();
  bool getHoodPosition();
  void flipHoodUp(bool location);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX m_shooterMotor1;
  WPI_TalonFX m_shooterMotor2;
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_feederMotor;
  frc::Solenoid m_hoodAdjuster;
};
