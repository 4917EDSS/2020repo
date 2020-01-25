/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeCmd.h"

IntakeCmd::IntakeCmd(IntakeSub* subsystem) : m_IntakeSub(subsystem){
  // Use addRequirements() here to declare subsystem dependencies.
AddRequirements({subsystem});
}

// Called when the command is initially scheduled.
void IntakeCmd::Initialize() {
  m_IntakeSub->setIntake(0.6);
}

// Called once the command ends or is interrupted.
void IntakeCmd::End(bool interrupted) {
    m_IntakeSub->setIntake(0);
}
