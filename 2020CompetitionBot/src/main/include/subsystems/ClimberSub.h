/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <ctre/Phoenix.h>

class ClimberSub : public frc2::SubsystemBase {
 public:
  ClimberSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void init();
  void Periodic();
  void releaseLatch(bool position);
  void setWinchPower(double speed);
  void moveOnGenSwitch(double power);
  double getArmMotorEncoderRaw();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_armMotor;
  frc::Solenoid m_climbReleaseLatch; 
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_climbBalanceMotor;
};
