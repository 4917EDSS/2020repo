/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetHoodSpeedCmd.h"

SetHoodSpeedCmd::SetHoodSpeedCmd(ShooterSub* shooterSub, double speed) : m_shooterSub(shooterSub), m_speed(speed) {
  AddRequirements({shooterSub});
}

// Called when the command is initially scheduled.
void SetHoodSpeedCmd::Initialize() {
  m_shooterSub->setHoodSpeed(m_speed);
}

// Called repeatedly when this Command is scheduled to run
void SetHoodSpeedCmd::Execute() {}

// Called once the command ends or is interrupted.
void SetHoodSpeedCmd::End(bool interrupted) {
  m_shooterSub->setHoodSpeed(0);
}

// Returns true when the command should end.
bool SetHoodSpeedCmd::IsFinished() { return false; }
