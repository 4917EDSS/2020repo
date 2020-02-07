/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeCmd.h"

IntakeCmd::IntakeCmd(IntakeSub* subsystem) : m_intakeSub(subsystem){
  // Use addRequirements() here to declare subsystem dependencies.
AddRequirements({subsystem});
}

// Called when the command is initially scheduled.
void IntakeCmd::Initialize() {
  m_intakeSub->setIntake(1);
}

void IntakeCmd::Execute() {
  if(m_intakeSub->getBallIntakeSensor()) {
    // When intake sensor is true, intake until first ball sensor is false then true again
  }
}

// Called once the command ends or is interrupted.
void IntakeCmd::End(bool interrupted) {
  m_intakeSub->setIntake(0);

}
