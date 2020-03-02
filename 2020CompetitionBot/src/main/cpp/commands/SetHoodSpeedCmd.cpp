/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetHoodSpeedCmd.h"

constexpr double kMaxHoodPower = 0.2;
SetHoodSpeedCmd::SetHoodSpeedCmd(ShooterSub* shooterSub, frc::Joystick* joystick)
  : m_shooterSub(shooterSub),
    m_joystick(joystick) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooterSub});
}

// Called when the command is initially scheduled.
void SetHoodSpeedCmd::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void SetHoodSpeedCmd::Execute() {
  double power = m_joystick->GetY();
  if (power >= kMaxHoodPower) {
    power = kMaxHoodPower;
  }
  else if (power <= -kMaxHoodPower) {
    power = -kMaxHoodPower;
  }
  m_shooterSub->setHoodSpeed(power);
}

// Called once the command ends or is interrupted.
void SetHoodSpeedCmd::End(bool interrupted) {
  m_shooterSub->setHoodSpeed(0);
}

// Returns true when the command should end.
bool SetHoodSpeedCmd::IsFinished() { return false; }
