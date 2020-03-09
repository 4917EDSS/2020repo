/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include "commands/ClimbReleaseArmCmd.h"
#include "Constants.h"

ClimbReleaseArmCmd::ClimbReleaseArmCmd(ClimberSub* climbSub, frc::Joystick* joystick)
  : m_climbSub(climbSub),
    m_joystick(joystick) {
  
  AddRequirements({climbSub});
}

// Called when the command is initially scheduled.
void ClimbReleaseArmCmd::Initialize() {
  if (m_climbSub->getOperatorShiftState(m_joystick) == DpadConstants::kDown) {
    m_climbSub->releaseLatch(true);
  } else if (m_climbSub->getOperatorShiftState(m_joystick) == DpadConstants::kUp) {
    m_climbSub->releaseLatch(false);
  }
}

// Returns true when the command should end.
bool ClimbReleaseArmCmd::IsFinished() { return true; }
