/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DriveWithJoystickCmd.h"

DriveWithJoystickCmd::DriveWithJoystickCmd(DrivetrainSub *drivetrainSub, frc::Joystick *driverController) 
      : m_drivetrainSubPtr(drivetrainSub), 
        m_driverControllerPtr(driverController) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_drivetrainSubPtr);
}

// Called when the command is initially scheduled.
void DriveWithJoystickCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystickCmd::Execute() {
  m_drivetrainSubPtr->drive(m_driverControllerPtr->GetX(), m_driverControllerPtr->GetZ());
}

// Called once the command ends or is interrupted.
void DriveWithJoystickCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveWithJoystickCmd::IsFinished() { return false; }
