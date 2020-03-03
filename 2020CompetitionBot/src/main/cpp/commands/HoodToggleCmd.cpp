/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/HoodToggleCmd.h"
#include "subsystems/ShooterSub.h"

HoodToggleCmd::HoodToggleCmd(ShooterSub* shooterSub) 
: m_ShooterSub(shooterSub)
{
  AddRequirements({shooterSub});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void HoodToggleCmd::Initialize() {
  m_ShooterSub->flipHoodUp(not m_ShooterSub->getHoodPosition());
}

// Called repeatedly when this Command is scheduled to run
void HoodToggleCmd::Execute() {}

// Called once the command ends or is interrupted.
void HoodToggleCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool HoodToggleCmd::IsFinished() { return true; }
