/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/WPILib.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>

class ClimberSub : public frc2::SubsystemBase {
 public:
  ClimberSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

 private:
 std::shared_ptr <rev::CANSparkMax> elevatorMotor1;
 std::shared_ptr <frc::Solenoid> climbReleaseSolenoid;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
