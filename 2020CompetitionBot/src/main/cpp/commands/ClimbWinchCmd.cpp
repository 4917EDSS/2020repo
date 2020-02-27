/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/ClimbWinchCmd.h"
#include "Constants.h"

ClimbWinchCmd::ClimbWinchCmd(ClimberSub* climbSub, bool isUp)
  : m_climbSub(climbSub),
    m_isUp(isUp)
  {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climbSub});
  
  frc::SmartDashboard::PutNumber("ClimbWinchPower", ClimbConstants::kClimbWinchPower);
}

// Called when the command is initially scheduled.
// We just turn the power on and leave it until command is interrupted calling End
void ClimbWinchCmd::Initialize() {
  // Determine the power to apply to motor
  double p = frc::SmartDashboard::GetNumber("ClimbWinchPower", ClimbConstants::kClimbWinchPower);
  if (m_isUp) {
    p = -p;
  }

  // Apply power
  m_climbSub->setWinchPower(p);
}

// Called repeatedly when this Command is scheduled to run
void ClimbWinchCmd::Execute() {
}

// Relying on command inrerruption to end this
void ClimbWinchCmd::End() {
  // Turn off motor when this command is interrupted
  m_climbSub->setWinchPower(0.0);
}

// Returns true when the command should end.
bool ClimbWinchCmd::IsFinished() { return false; }
