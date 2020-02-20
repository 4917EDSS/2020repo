/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TurnControlPanelToColourCmd.h"
#include "subsystems/ControlPanelSub.h"
#include "Constants.h"
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>

TurnControlPanelToColourCmd::TurnControlPanelToColourCmd() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void TurnControlPanelToColourCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TurnControlPanelToColourCmd::Execute() {
    if (matchedColour == ControlPanelConstants::kBlueTarget) {
      colourString = "Blue";
    } else if (matchedColour == ControlPanelConstants::kRedTarget) {
      colourString = "Red";
    } else if (matchedColour == ControlPanelConstants::kGreenTarget) {
      colourString = "Green";
    } else if (matchedColour == ControlPanelConstants::kYellowTarget) {
      colourString = "Yellow";
    } else {
      colourString = "Unknown";
    }
}

// Called once the command ends or is interrupted.
void TurnControlPanelToColourCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool TurnControlPanelToColourCmd::IsFinished() { return false; }
