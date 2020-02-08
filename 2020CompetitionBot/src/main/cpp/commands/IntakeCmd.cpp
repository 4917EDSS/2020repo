/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeCmd.h"

IntakeCmd::IntakeCmd(IntakeSub* subsystem) : m_intakeSub(subsystem), m_state(0), m_startingEncDistance(0) { // Sensors need to be added to the command (PowerCellSensor1-4)
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({subsystem});
}

// Called when the command is initially scheduled.
void IntakeCmd::Initialize() {
  m_intakeSub->setFrontRollerIntakePower(1.0);
}

void IntakeCmd::Execute() {
  if (!m_intakeSub->getMagazineFullSensor() && !m_intakeSub->getFrontIntakeSensor())
    m_state = 0;
  switch (m_state)
  {
  case 0: // state 0, waiting for a new ball to come into the robot. Magazine isn't full
    if(m_intakeSub->getFrontIntakeSensor()) {
       m_intakeSub->setMagazineIntakePower(1.0);
      //  m_startingEncDistance = m_intakeSub->getEncoderDistance();
      m_state = 1;
    }
    break;
  case 1:// state 1, currently in the process of storing the ball inside the magazine. 
    if (m_intakeSub->getFrontIntakeSensor() && !m_intakeSub->getMagazineFullSensor()) {
      // double currentEncDistance = m_intakeSub->getEncoderDistance();
      m_intakeSub->setMagazineIntakePower(1.0);
      // if ((currentEncDistance - m_startingEncDistance) >= ktargetDistance) {
      //   m_intakeSub->setMagazineIntakePower(0.0);
      // }
    }
    if (m_intakeSub->getMagazineFullSensor())
      m_state = 2;
    break;
  case 2:
    if (m_intakeSub->getMagazineFullSensor()) {
      m_intakeSub->setMagazineIntakePower(0.0);
    }
  
  default:
    break;
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
