/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/ClimbBalanceCmd.h"
#include "Constants.h"

ClimbBalanceCmd::ClimbBalanceCmd(ClimberSub* climbSub, bool isRight)
  : m_climbSub(climbSub),
    m_isRight(isRight)
  {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climbSub});

  frc::SmartDashboard::PutNumber("MoveOnGenSwitchPower", ClimbConstants::kMoveOnGenSwitchPower);
}

// Called when the command is initially scheduled.
// We just turn the power on and leave it until command is interrupted calling End
void ClimbBalanceCmd::Initialize() {
  // Determine the power to apply to motor
  double p = frc::SmartDashboard::GetNumber("MoveOnGenSwitchPower", ClimbConstants::kMoveOnGenSwitchPower);
  if (m_isRight) {
    p = -p;
  }

  // Apply power
    m_climbSub->moveOnGenSwitch(p);
}

// Called repeatedly when this Command is scheduled to run
void ClimbBalanceCmd::Execute() {
}

// Relying on command inrerruption to end this
void ClimbBalanceCmd::End(bool interrupted) {
  // Turn off motor when this command is interrupted
  m_climbSub->moveOnGenSwitch(0.0);
}

// Returns true when the command should end.
bool ClimbBalanceCmd::IsFinished() { return false; }
