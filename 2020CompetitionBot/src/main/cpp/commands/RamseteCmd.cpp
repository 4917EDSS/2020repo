/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/RamseteCmd.h"
#include <frc/controller/SimpleMotorFeedForward.h>
#include <iostream>

RamseteCmd::RamseteCmd(Trajectory t, DrivetrainSub* drivetrainSub, frc2::PIDController leftController, frc2::PIDController rightController)
  : frc2::RamseteCommand(t, 
      [drivetrainSub]() {return drivetrainSub->getPose();},
      frc::RamseteController(AutoConstants::kRamseteB, AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [drivetrainSub] {return drivetrainSub->getWheelSpeeds();},
      leftController,
      rightController,
      [=](auto left, auto right) {
        drivetrainSub->tankDriveVolts(left, right);
        std::cout << "L Target: " << leftController.GetSetpoint() << " a "<< drivetrainSub->getWheelSpeeds().left << std::endl;
        std::cout << "R Target: " << rightController.GetSetpoint() << " a "<< drivetrainSub->getWheelSpeeds().right << std::endl;
      },
      {drivetrainSub}),
    m_drivetrainSub(drivetrainSub) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrainSub});
}

void RamseteCmd::End(bool interrupted) {
  frc2::RamseteCommand::End(interrupted);
  m_drivetrainSub->tankDrive(0, 0);
}
