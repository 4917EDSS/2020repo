/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TurnControlPanelThreeTimesCmd.h"
#include "constants.h"

TurnControlPanelThreeTimesCmd::TurnControlPanelThreeTimesCmd(ControlPanelSub* controlPanelSub)
  : m_controlPanelSub(controlPanelSub) {
  
  AddRequirements({controlPanelSub});
}

// Called when the command is initially scheduled.
void TurnControlPanelThreeTimesCmd::Initialize() {
  // we probably have to wait before we read the colour
  m_startingColour = m_controlPanelSub->getColour();
  m_controlPanelSub->setWheelPower(ControlPanelConstants::kMaxWheelSpeed);
  m_inExpectedColour = true;
}

// Called repeatedly when this Command is scheduled to run
void TurnControlPanelThreeTimesCmd::Execute() {
  frc::Color currentColour = m_controlPanelSub->getColour();
  if(!m_inExpectedColour && (currentColour == m_startingColour)) {
    m_inExpectedColour = true;
    m_numHalfRotations = m_numHalfRotations + 1;
    // printf("[A]TurnControlPanelThreeTimesCmd.numHalfRotations = %d \n", m_numHalfRotations);
    // printf("[A]TurnControlPanelThreeTimesCmd.inExpectedColour = %s \n", (m_inExpectedColour ? "True" : "False"));
  } 
  else if (m_inExpectedColour && !(currentColour == m_startingColour)) {
    m_inExpectedColour = false;
    // printf("[B]TurnControlPanelThreeTimesCmd.inExpectedColour = %s \n", m_inExpectedColour ? "True" : "False");
  }
}

// Called once the command ends or is interrupted.
void TurnControlPanelThreeTimesCmd::End(bool interrupted) {
  m_controlPanelSub->setWheelPower(0);
}

// Returns true when the command should end.
bool TurnControlPanelThreeTimesCmd::IsFinished() { 
  // We are done after 3 rotations (6 half rotations) and we have gone past the initial colour
  if((m_numHalfRotations >= 6) && !m_inExpectedColour) {
    // This may be too abrupt of a stop - needs tuning
    m_controlPanelSub->setWheelPower(0);
    return true;
  } 
  else {
    return false; 
  } 
}
