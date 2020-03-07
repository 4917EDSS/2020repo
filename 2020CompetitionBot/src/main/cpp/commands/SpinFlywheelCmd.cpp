/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SpinFlywheelCmd.h"
#include "commands/ShootCmd.h"
#include "Constants.h"

//                          READ THIS
//                          READ THIS
// This command has to be used with shootCmd or the flywheel will never stop
//                          READ THIS
//                          READ THIS

SpinFlywheelCmd::SpinFlywheelCmd(ShooterSub* shooterSub, bool isFar) 
  : m_shooterSub(shooterSub),
    m_isFar(isFar),
    m_targetSpeed(0) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooterSub});
}

// Called when the command is initially scheduled.
void SpinFlywheelCmd::Initialize() {
  m_targetSpeed = (m_isFar ? ShooterConstants::kFarTargetSpeed : ShooterConstants::kCloseTargetSpeed);
  double feed = m_targetSpeed / ShooterConstants::kMaxRPM;
  m_shooterSub->setPower(feed);
}

// Called repeatedly when this Command is scheduled to run
void SpinFlywheelCmd::Execute() {}

// Called once the command ends or is interrupted.
void SpinFlywheelCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool SpinFlywheelCmd::IsFinished() { 
  if(m_shooterSub->getSpeed() >= (m_targetSpeed * 0.9)) {
    return true;
  }
  else {
    return false;
  }
}
