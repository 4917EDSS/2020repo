/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ExpelCmd.h"

ExpelCmd::ExpelCmd(IntakeSub* intakeSub) 
  : m_intakeSub(intakeSub){

  AddRequirements({intakeSub});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ExpelCmd::Initialize() {
  m_intakeSub->setFrontRollerIntakePower(-1.0);
  m_intakeSub->setMagazineIntakePower(-1.0);
}

// Called repeatedly when this Command is scheduled to run
void ExpelCmd::Execute() {}

// Called once the command ends or is interrupted.
void ExpelCmd::End(bool interrupted) {
  m_intakeSub->setFrontRollerIntakePower(0.0);
  m_intakeSub->setMagazineIntakePower(0.0);
}

// Returns true when the command should end.
bool ExpelCmd::IsFinished() { return false; }
