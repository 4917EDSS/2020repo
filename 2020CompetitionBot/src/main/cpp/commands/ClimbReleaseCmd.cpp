/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ClimbReleaseCmd.h"

ClimbReleaseCmd::ClimbReleaseCmd(ClimberSub* subsystem)  : m_ClimbSub(subsystem){
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ClimbReleaseCmd::Initialize() {
  m_ClimbSub->togglePosition(1);
}

// Called repeatedly when this Command is scheduled to run
void ClimbReleaseCmd::Execute() {
} 

// Called once the command ends or is interrupted.
void ClimbReleaseCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimbReleaseCmd::IsFinished() { return true; }
