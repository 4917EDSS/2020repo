/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeCmd.h"

IntakeCmd::IntakeCmd(IntakeSub* intakeSub, DrivetrainSub* drivetrainSub)
  : m_intakeSub(intakeSub),
    m_drivetrainSub(drivetrainSub),
    m_state(0),
    m_startingEncDistance(0) {
  
  AddRequirements({intakeSub});
}

// Called when the command is initially scheduled.
void IntakeCmd::Initialize() {
  m_state = 0;
  m_intakeSub->setFrontRollerIntakePower(1.0);
  m_drivetrainSub->shiftDown();
  m_drivetrainSub->disableAutoShift();
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
  case 1: // state 1, wait for magazine front sensor to see the ball
    if(m_intakeSub->getMagazineFrontSensor()) {
      m_state = 2;
    }
    break;
  case 2: // state 2, currently in the process of storing the ball inside the magazine. 
    if(!m_intakeSub->getMagazineFrontSensor()) {
      m_intakeSub->setMagazineIntakePower(0.0);
      m_state = 0;
    }
    break;
  default: 
    m_intakeSub->setMagazineIntakePower(0.0);
    break;
  }
}

bool IntakeCmd::IsFinished() { 
  if(m_intakeSub->getFrontIntakeSensor() && m_intakeSub->getMagazineFullSensor()) {
    // There are 5 powercells in the robot 
    return true;
  }
  return false; 
}

// Called once the command ends or is interrupted.
void IntakeCmd::End(bool interrupted) {
  m_intakeSub->setFrontRollerIntakePower(0.0);
  m_intakeSub->setMagazineIntakePower(0.0);
  m_drivetrainSub->enableAutoShift();
}
