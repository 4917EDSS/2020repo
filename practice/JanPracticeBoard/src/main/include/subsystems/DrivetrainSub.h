/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>

class DrivetrainSub : public frc2::SubsystemBase {
 public:
  DrivetrainSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  void drive(double lPower, double rPower);
  void setAuxPower(double power);
  void resetEncoder();
  double getLeftEncoderPosition();
  double getRightEncoderPosition();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_leftMotor1;
  rev::CANSparkMax m_rightMotor1;
  WPI_VictorSPX m_auxMotor;   // Not really a drivetrain motor but this is just practice
  
};
