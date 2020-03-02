/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ToggleControlPanelArmCmd.h"

ToggleControlPanelArmCmd::ToggleControlPanelArmCmd() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ToggleControlPanelArmCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ToggleControlPanelArmCmd::Execute() {}

// Called once the command ends or is interrupted.
void ToggleControlPanelArmCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool ToggleControlPanelArmCmd::IsFinished() { return false; }
