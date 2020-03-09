/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/RamseteCmd.h"
#include <frc/controller/SimpleMotorFeedForward.h>
#include <iostream>

constexpr double kRamseteB = 2;
constexpr double kRamseteZeta = 0.7;
constexpr double kPDriveVel = 2.43;

RamseteCmd::RamseteCmd(Trajectory t, DrivetrainSub* drivetrainSub)
  : frc2::RamseteCommand(t, 
      [drivetrainSub]() {return drivetrainSub->getPose();},
      frc::RamseteController(kRamseteB, kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [drivetrainSub] {return drivetrainSub->getWheelSpeeds();},
      frc2::PIDController(kPDriveVel, 0, 0),
      frc2::PIDController(kPDriveVel, 0, 0),
      [=](auto left, auto right) {
        drivetrainSub->tankDriveVolts(left, right);
      },
      {drivetrainSub}),
    m_drivetrainSub(drivetrainSub) {
  
  AddRequirements({drivetrainSub});
}

void RamseteCmd::End(bool interrupted) {
  frc2::RamseteCommand::End(interrupted);
  m_drivetrainSub->tankDrive(0, 0);
  std::cout << "X pos: " << m_drivetrainSub->getPose().Translation().X() << "\n";
  std::cout << "Y pos: " << m_drivetrainSub->getPose().Translation().Y() << "\n";
  }
