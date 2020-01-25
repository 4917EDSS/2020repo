/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ClimbWinchCmd.h"

ClimbWinchCmd::ClimbWinchCmd(ClimberSub* climbSub) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called repeatedly when this Command is scheduled to run
void ClimbWinchCmd::Execute() {
  m_climbSub->setWinchSpeed(1);
}

void ClimbWinchCmd::End(){
  m_climbSub->setWinchSpeed(0);
}
// Returns true when the command should end.
bool ClimbWinchCmd::IsFinished() { return false; }
