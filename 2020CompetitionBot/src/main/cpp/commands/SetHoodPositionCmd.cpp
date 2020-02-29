/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetHoodPositionCmd.h"
constexpr double kP=0.2;
constexpr double kTolerance=50.0;
constexpr double kLowHood = 14000;
constexpr double kControlStartDelta = 999999.9;
constexpr double kMinPower = 0.125;
constexpr double kMaxPower = 0.2;


SetHoodPositionCmd::SetHoodPositionCmd(ShooterSub* shooterSub, double targetPosition)
  : m_shooterSub(shooterSub),
    m_targetPosition(targetPosition),
    m_pc(targetPosition, kControlStartDelta, kTolerance, kMinPower, kMaxPower) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooterSub});
}

// Called when the command is initially scheduled.
void SetHoodPositionCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetHoodPositionCmd::Execute() {
  double outputPower = (m_pc.getPower(m_shooterSub->getHoodEncoder()));
  m_shooterSub->setHoodSpeed(outputPower);
  std::cout << "Executed " << outputPower << "\n";
  // m_shooterSub->setHoodSpeed(0.05);
}

// Called once the command ends or is interrupted.
void SetHoodPositionCmd::End(bool interrupted) {
  m_shooterSub->setHoodSpeed(0.0);
}

// Returns true when the command should end.
bool SetHoodPositionCmd::IsFinished() {
  // if hood is within +-tolerence of target and it's not going too fast, then we're done
  if ((fabs(m_shooterSub->getHoodEncoder() - m_targetPosition) <= kTolerance) /*&& (currentVelocity < kTargetVelocity), if necessary*/) {
    std::cout << "Finished \n";
    return true;
  }
  return false;
}

