/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetHoodPositionCmd.h"
constexpr double kP=2.0;
constexpr double kTolerance=50.0;

SetHoodPositionCmd::SetHoodPositionCmd(ShooterSub* shooterSub, double targetPosition) : m_shooterSub(shooterSub), m_targetPosition(targetPosition) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooterSub});
}

// Called when the command is initially scheduled.
void SetHoodPositionCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetHoodPositionCmd::Execute() {
  double diff=m_targetPosition-m_shooterSub->getHoodEncoder();
  m_shooterSub->setHoodSpeed(diff*kP);
}

// Called once the command ends or is interrupted.
void SetHoodPositionCmd::End(bool interrupted) {
  m_shooterSub->setHoodSpeed(0.0);
}

// Returns true when the command should end.
bool SetHoodPositionCmd::IsFinished() {
  if (m_shooterSub->getHoodEncoder() >= m_targetPosition-kTolerance and m_shooterSub->getHoodEncoder() <= m_targetPosition+kTolerance) {
    return true;
  }
  return false;
}
