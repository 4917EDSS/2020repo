/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeCmd.h"

IntakeCmd::IntakeCmd(IntakeSub* subsystem) : m_intakeSub(subsystem), m_state(0) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({subsystem});
}

// Called when the command is initially scheduled.
void IntakeCmd::Initialize() {
  m_intakeSub->setFrontRollerIntakePower(1.0);
}

void IntakeCmd::Execute() {
  switch (m_state)
  {
  case 0:
    if(m_intakeSub->getFrontIntakeSensor()) {
       m_intakeSub->setMagazineIntakePower(1.0);
      m_state = 1;
    }
    break;
  
  default:
    break;
  }
  if(m_intakeSub->getFrontIntakeSensor()) {
    if(!m_intakeSub->getMagazineFullSensor()) {
      // Move the ball in 1 position
    }
    else {
    
    }
    
  }
}

bool IntakeCmd::IsFinished() { 
  if(m_intakeSub->getFrontIntakeSensor() && m_intakeSub->getMagazineFullSensor()) {
    return true;
  }
  return false; 
}

// Called once the command ends or is interrupted.
void IntakeCmd::End(bool interrupted) {
  m_intakeSub->setFrontRollerIntakePower(0.0);
  m_intakeSub->setMagazineIntakePower(0.0);
}
