/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ClimbReleaseCmd.h"

ClimbReleaseCmd::ClimbReleaseCmd(ClimberSub* climbSub)
  : m_ClimbSub(climbSub)
  {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climbSub});
}

// Called when the command is initially scheduled.
void ClimbReleaseCmd::Initialize() {
  m_ClimbSub->releaseLatch(true);
}

// Returns true when the command should end.
bool ClimbReleaseCmd::IsFinished() { return true; }
