/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DisableAutoShiftCmd.h"
#include "subsystems/DrivetrainSub.h"

DisableAutoShiftCmd::DisableAutoShiftCmd(DrivetrainSub* drivetrainSub)
  : m_drivetrainSub(drivetrainSub) {
  
  AddRequirements({drivetrainSub});
}

// Called when the command is initially scheduled.
void DisableAutoShiftCmd::Initialize() {
  m_drivetrainSub->disableAutoShift();
  m_drivetrainSub->shiftDown();
}

// Called repeatedly when this Command is scheduled to run
void DisableAutoShiftCmd::Execute() {}

// Called once the command ends or is interrupted.
void DisableAutoShiftCmd::End(bool interrupted) {
  m_drivetrainSub->enableAutoShift();
}

// Returns true when the command should end.
bool DisableAutoShiftCmd::IsFinished() { return false; }