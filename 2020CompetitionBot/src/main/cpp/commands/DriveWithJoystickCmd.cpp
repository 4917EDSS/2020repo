/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include "commands/DriveWithJoystickCmd.h"

DriveWithJoystickCmd::DriveWithJoystickCmd(DrivetrainSub* drivetrainSub, frc::Joystick* joystick) : m_drivetrainSub(drivetrainSub), m_joystick(joystick){
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrainSub});
}

// Called when the command is initially scheduled.
void DriveWithJoystickCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystickCmd::Execute() {
  printf("in execute\n");
  m_drivetrainSub->arcadeDrive(
          m_joystick->GetY(),
          -m_joystick->GetZ());
          }

// Called once the command ends or is interrupted.
void DriveWithJoystickCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveWithJoystickCmd::IsFinished() { return false; }
