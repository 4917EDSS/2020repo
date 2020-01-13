/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/PrintHelloCmd.h"

PrintHelloCmd::PrintHelloCmd() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void PrintHelloCmd::Initialize() {std::cout << "Hello from PrintHelloCmd\n";}

// Called repeatedly when this Command is scheduled to run
void PrintHelloCmd::Execute() {}

// Called once the command ends or is interrupted.
void PrintHelloCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool PrintHelloCmd::IsFinished() { return true; }
