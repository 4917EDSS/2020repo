/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include "commands/TurnControlPanelToColourCmd.h"
#include "subsystems/ControlPanelSub.h"
#include "Constants.h"

TurnControlPanelToColourCmd::TurnControlPanelToColourCmd(ControlPanelSub* controlPanelSub)
  : m_controlPanelSub(controlPanelSub) {
  
  AddRequirements({controlPanelSub});
}

// Called when the command is initially scheduled.
void TurnControlPanelToColourCmd::Initialize() {
  if(m_controlPanelSub->isColourToTurnToValid()) {
    m_colourToTurnTo = m_controlPanelSub->getColourToTurnTo();
    m_controlPanelSub->setWheelPower(ControlPanelConstants::kMaxWheelSpeed);
  }
  else {
    // We don't have a colour from the field yet so quit this command (see IsFinished)
    m_colourToTurnToValid = false;
  }
}

// Called repeatedly when this Command is scheduled to run
void TurnControlPanelToColourCmd::Execute() {
  m_currentColour = m_controlPanelSub->getColour();
}

// Called once the command ends or is interrupted.
void TurnControlPanelToColourCmd::End(bool interrupted) {
  m_controlPanelSub->setWheelPower(0);
}

// Returns true when the command should end.
bool TurnControlPanelToColourCmd::IsFinished() {
  if(!m_colourToTurnToValid || (m_currentColour == m_colourToTurnTo)) {
    return true;
  } 
  else {
    return false;
  }
}

