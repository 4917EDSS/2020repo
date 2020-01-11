/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AuxMotorReverseCmd.h"

AuxMotorReverseCmd::AuxMotorReverseCmd(DrivetrainSub *drivetrainSub, double power, double time)
      : m_drivetrainSubPtr(drivetrainSub),
        m_power(power),
        m_time(time) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AuxMotorReverseCmd::Initialize() {
  std::cout << "Starting AuxMotorReverseCmd\n";
  m_drivetrainSubPtr->setAuxPower(m_power);
}

// Called repeatedly when this Command is scheduled to run
void AuxMotorReverseCmd::Execute() {}

// Called once the command ends or is interrupted.
void AuxMotorReverseCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool AuxMotorReverseCmd::IsFinished() { 
  // TODO: This should run on a timer either internally or as a WithTimeout decorator to the command in RobotContainer.cpp
  return true; 
}
