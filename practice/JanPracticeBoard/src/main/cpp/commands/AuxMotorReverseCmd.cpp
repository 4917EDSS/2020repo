/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AuxMotorReverseCmd.h"

  double m_time;
AuxMotorReverseCmd::AuxMotorReverseCmd(DrivetrainSub *drivetrainSub, double power)
      : m_drivetrainSubPtr(drivetrainSub),
        m_power(power) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AuxMotorReverseCmd::Initialize() {
  std::cout << "Starting AuxMotorReverseCmd with " << m_power << " power.\n";
  m_drivetrainSubPtr->setAuxPower(m_power);
}

// Called repeatedly when this Command is scheduled to run
void AuxMotorReverseCmd::Execute() {}

// Called once the command ends or is interrupted.
void AuxMotorReverseCmd::End(bool interrupted) {
  m_drivetrainSubPtr->setAuxPower(0);
}

// Returns true when the command should end.
bool AuxMotorReverseCmd::IsFinished() { 
  // This command is meant to be used with a WithTimeout(?_s) decorator
  return false; 
}
