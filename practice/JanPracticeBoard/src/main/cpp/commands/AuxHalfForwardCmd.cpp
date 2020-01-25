/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AuxHalfForwardCmd.h"

AuxHalfForwardCmd::AuxHalfForwardCmd(DrivetrainSub *drivetrainsubPtr) 
   : m_drivetrainSubPtr(drivetrainsubPtr) {
  
  AddRequirements(m_drivetrainSubPtr);
}

// Called when the command is initially scheduled.
void AuxHalfForwardCmd::Initialize() {
  std::cout << "this is the AuxHalfForwardCmd/n";
  m_drivetrainSubPtr->setAuxPower(0.5);
}

// Called repeatedly when this Command is scheduled to run
void AuxHalfForwardCmd::Execute() {}

// Called once the command ends or is interrupted.
void AuxHalfForwardCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool AuxHalfForwardCmd::IsFinished() { return true; }