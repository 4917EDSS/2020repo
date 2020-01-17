/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
 #include <ctre/Phoenix.h>

class IntakeSub : public frc2::SubsystemBase {
 public:
  IntakeSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

 private:

  
  std::shared_ptr<ctre::phoenix::motorcontrol::can::WPI_VictorSPX>FrontTopIntakeMotor;
  std::shared_ptr<ctre::phoenix::motorcontrol::can::WPI_VictorSPX>FrontBottomIntakeMotor;
  std::shared_ptr<ctre::phoenix::motorcontrol::can::WPI_VictorSPX>BackTopIntakeMotor;
  std::shared_ptr<ctre::phoenix::motorcontrol::can::WPI_VictorSPX>ConveyorBeltIntakeMotor;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
