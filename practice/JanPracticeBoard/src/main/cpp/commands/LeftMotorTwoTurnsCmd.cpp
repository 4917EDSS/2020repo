/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/LeftMotorTwoTurnsCmd.h"

// -----------------------------------------------------------------------------
// Command to turn the left motor 2 turns.
// -----------------------------------------------------------------------------
LeftMotorTwoTurnsCmd::LeftMotorTwoTurnsCmd(DrivetrainSub *drivetrainSub)
  : m_drivetrainSubPtr(drivetrainSub) {

  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_drivetrainSubPtr);
}

// -----------------------------------------------------------------------------
// Called when the command is initially scheduled.
// -----------------------------------------------------------------------------
void LeftMotorTwoTurnsCmd::Initialize() {
  std::cout << "LeftMotorTwoTurnsCmd: Start\n";

  // /Reset encoder for good measure.
  m_drivetrainSubPtr->resetEncoder();
  
  // Initialize turn tracking member variable.
  m_turnsAchieved = 0.0;

  // Set initial power.
  m_drivetrainSubPtr->drive(kInitialPower, 0.0);
}

// -----------------------------------------------------------------------------
// Called repeatedly (50 times per sec) when this Command is scheduled to run.
// -----------------------------------------------------------------------------
void LeftMotorTwoTurnsCmd::Execute() {
  // Determine the left motor position and the turns achieved.
  double leftPos = m_drivetrainSubPtr->getLeftEncoderPosition();
  m_turnsAchieved = leftPos / kTicksPerTurn;

  // Debug tracing. May make sense to comment out later.
  std::cout << "LeftMotorTwoTurnsCmd: Execute - Turns Achieved = ";
  printf("%5.3f", m_turnsAchieved);
  std::cout << "; Left Position = ";
  std::cout << leftPos;
  std::cout << "\n";

  // Dial back the power as we approach 2 turns.
  if (m_turnsAchieved >= 1.75)
    // Once 1.75 turns are acheived, set power to 1/3 initial power.
    m_drivetrainSubPtr->drive(kInitialPower / 3, 0.0);
  else if (m_turnsAchieved >= 1.0)
    // Once 1 turn is acheived, set power to 1/2 initial power.
    m_drivetrainSubPtr->drive(kInitialPower / 2, 0.0);
}

// -----------------------------------------------------------------------------
// Called once the command ends or is interrupted.
// -----------------------------------------------------------------------------
void LeftMotorTwoTurnsCmd::End(bool interrupted) {
  std::cout << "LeftMotorTwoTurnsCmd: End\n";
  
  // Abruptly shut down motor once 2 turns have been acheived.
  // NB: This is likely not appropriate with real robot weight and momentum
  //     unless the Execute method dialed back the power enough already.
  m_drivetrainSubPtr->drive(0.0, 0.0);
  
  // /Reset encoder for good measure.
  // TODO: Would this ever be useful?
  m_drivetrainSubPtr->resetEncoder();
}

// -----------------------------------------------------------------------------
// Returns true when the command should end.
// -----------------------------------------------------------------------------
bool LeftMotorTwoTurnsCmd::IsFinished() {
  // Debug tracing. May make sense to comment out later.
  std::cout << "LeftMotorTwoTurnsCmd: IsFinished - Turns achieved = ";
  printf("%5.3f", m_turnsAchieved);
  std::cout << "\n";

  // We're done once 2 turns have been acheived, but done also if more in case
  // our math is off.
  if (m_turnsAchieved >= 2.0)
  {
    // Done.
    return true;
  } else {
    // Not done yet.
    return false;
  }
}
