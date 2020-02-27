/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/RamseteCmd.h"
#include <frc/controller/SimpleMotorFeedForward.h>

// TODO: Check that '[&drivetrainSub]' is correct.  I might need to be '[drivetrainSub]' since it's alreayd a pointer.
RamseteCmd::RamseteCmd(Trajectory t, DrivetrainSub* drivetrainSub)
  : frc2::RamseteCommand(t, 
      [&drivetrainSub]() {return drivetrainSub->getPose();},
      frc::RamseteController(AutoConstants::kRamseteB, AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [&drivetrainSub] {return drivetrainSub->getWheelSpeeds();},
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      [&drivetrainSub](auto left, auto right) {drivetrainSub->tankDriveVolts(left, right);},
      {drivetrainSub}),
    m_drivetrainSub(drivetrainSub)
  {
  // Use addRequirements() here to declare subsystem dependencies.
}

void RamseteCmd::End(bool interrupted) {
  frc2::RamseteCommand::End(interrupted);
  m_drivetrainSub->tankDrive(0, 0);
}