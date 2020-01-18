/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShootCmd.h"



ShootCmd::ShootCmd(ShooterSub* subsystem) : m_ShootSub(subsystem){
  // Use addRequirements() here to declare subsystem dependencies.
AddRequirements({subsystem});
}

// Called when the command is initially scheduled.
void ShootCmd::Initialize() {
  m_ShootSub->setSpeed(0.6);
}

// Called once the command ends or is interrupted.
void ShootCmd::End(bool interrupted) {
    m_ShootSub->setSpeed(0);
}
