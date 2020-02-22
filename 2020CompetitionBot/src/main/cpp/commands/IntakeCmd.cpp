/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeCmd.h"

IntakeCmd::IntakeCmd(IntakeSub* intakeSub) : m_intakeSub(intakeSub), m_state(0), m_startingEncDistance(0) { // Sensors need to be added to the command (PowerCellSensor1-4)
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({intakeSub});
}

// Called when the command is initially scheduled.
void IntakeCmd::Initialize() {
  m_state = 0;
  m_intakeSub->setFrontRollerIntakePower(1.0);
  //Annon's garbage trash that he made me put back in so we could run the stupid frigging practice bot
  //because our good bot is reliant on sensors to run, as it SHOULD be, but the dumb practice bot 
  //has no sensors because apparently we like overcomplicating things, so, delete this as soon as it's fixed, 
  //or just if you feel like it, I don't care, just delete it
  m_intakeSub->setMagazineIntakePower(1.0);
}

void IntakeCmd::Execute() {
  switch (m_state)
  {
  case 0: // state 0, waiting for a new ball to come into the robot. Magazine isn't full
    if(m_intakeSub->getFrontIntakeSensor()) {
      m_intakeSub->setMagazineIntakePower(1.0);
      m_state = 1;
    }
    break;
  case 1: // state 1, currently in the process of storing the ball inside the magazine. 
    if(!m_intakeSub->getFrontIntakeSensor()) {
      m_intakeSub->setMagazineIntakePower(0.0);
      m_state = 0;
    }
    break;
  case 2:
  break;
    default: m_intakeSub->setMagazineIntakePower(0.0);
    break;
  }
}

bool IntakeCmd::IsFinished() { 
  if(m_intakeSub->getFrontIntakeSensor() && m_intakeSub->getPowerCellSensor4()) {
    // There are 5 powercells in the robot 
    return true;
  }
  return false; 
}

// Called once the command ends or is interrupted.
void IntakeCmd::End(bool interrupted) {
  m_intakeSub->setFrontRollerIntakePower(0.0);
  m_intakeSub->setMagazineIntakePower(0.0);
}
