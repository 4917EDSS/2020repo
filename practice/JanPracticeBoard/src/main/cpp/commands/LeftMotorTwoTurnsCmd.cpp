/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/LeftMotorTwoTurnsCmd.h"

LeftMotorTwoTurnsCmd::LeftMotorTwoTurnsCmd(DrivetrainSub *drivetrainSub) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_drivetrainSubPtr->resetEncoder();  // TODO: should this be in Exectue instead?
}

// Called when the command is initially scheduled.
void LeftMotorTwoTurnsCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void LeftMotorTwoTurnsCmd::Execute() {
  std::cout << "LeftMotorTwoTurnsCmd: Start\n";
  m_turnsAchieved = 0;
  m_drivetrainSubPtr->drive(0.05, 0.0);
}

// Called once the command ends or is interrupted.
void LeftMotorTwoTurnsCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool LeftMotorTwoTurnsCmd::IsFinished() {

  double leftPos = m_drivetrainSubPtr->getLeftEncoderPosition();
  std::cout << "LeftMotorTwoTurnsCmd: Left Position = ";
  std::cout << leftPos;
  std::cout << "\n";
  m_turnsAchieved = (int)leftPos % kTicksPerTurn;

  // Abruptly shut down motor once 2 turns have been acheived
  // NB: This is likely not appropriate with real robot weight and momentum
  if (m_turnsAchieved == 2)
  {
      m_drivetrainSubPtr->drive(0.05, 0.0);
  }

  return true;
}
