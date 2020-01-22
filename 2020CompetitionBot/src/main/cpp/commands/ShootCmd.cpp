/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShootCmd.h"



ShootCmd::ShootCmd(ShooterSub* shooterSub,IntakeSub* intakeSub) : m_ShootSub(shooterSub),m_IntakeSub(intakeSub){
  // Use addRequirements() here to declare subsystem dependencies.
AddRequirements({shooterSub});
AddRequirements({intakeSub});
}

// Called when the command is initially scheduled.
void ShootCmd::Initialize() {
  m_ShootSub->setSpeed(0.6);
  m_IntakeSub->SetIntake(-0.6);
}

// Called once the command ends or is interrupted.
void ShootCmd::End(bool interrupted) {
    m_ShootSub->setSpeed(0);
    m_IntakeSub->SetIntake(0);


}
