/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include "commands/ClimbReleaseCmd.h"
#include "Constants.h"

ClimbReleaseCmd::ClimbReleaseCmd(ClimberSub* climbSub, frc::Joystick* joystick)
  : m_climbSub(climbSub),
    m_joystick(joystick)
  {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climbSub});
}

// Called when the command is initially scheduled.
void ClimbReleaseCmd::Initialize() {
  if (m_climbSub->getOperatorShiftState(m_joystick) == DpadConstants::kDown) {
    m_climbSub->releaseLatch(true);
  }
}

// Returns true when the command should end.
bool ClimbReleaseCmd::IsFinished() { return true; }
