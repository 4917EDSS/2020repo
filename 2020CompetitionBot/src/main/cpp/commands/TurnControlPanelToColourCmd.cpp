/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/DriverStation.h>
#include "commands/TurnControlPanelToColourCmd.h"
#include "subsystems/ControlPanelSub.h"
#include "Constants.h"

TurnControlPanelToColourCmd::TurnControlPanelToColourCmd(ControlPanelSub* controlPanelSub)
  : m_controlPanelSub(controlPanelSub) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({controlPanelSub});
}

// Called when the command is initially scheduled.
void TurnControlPanelToColourCmd::Initialize() {
  DetermineColourToTurnTo();
  m_controlPanelSub->flipArmUp(true);
  // we probably have to wait before we read the colour
  m_startingColour = m_controlPanelSub->getColour();
  m_controlPanelSub->setWheelPower(ControlPanelConstants::kMaxWheelSpeed);
}

// Called repeatedly when this Command is scheduled to run
void TurnControlPanelToColourCmd::Execute() {
  m_currentColour = m_controlPanelSub->getColour();
}

// Called once the command ends or is interrupted.
void TurnControlPanelToColourCmd::End(bool interrupted) {
  m_controlPanelSub->flipArmUp(false);
}

// Returns true when the command should end.
bool TurnControlPanelToColourCmd::IsFinished() {
  if(m_currentColour == m_ColourToTurnTo) {
    // This may be too abrupt of a stop - needs tuning
    m_controlPanelSub->setWheelPower(0);
    return true;
  } 
  else {
    return false;
  }
}

// TODO:  Move this to the Control Panel subsystem
void TurnControlPanelToColourCmd::DetermineColourToTurnTo() {
  // Get colour from the field
  std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

  if(gameData.length() > 0) {
    switch (gameData[0]) {
      case 'G' :
        //Green case code
        m_ColourToTurnTo = ControlPanelConstants::kYellowTarget;
        break;
      case 'B' :
        //Blue case code
        m_ColourToTurnTo = ControlPanelConstants::kRedTarget;
        break;
      case 'Y' :
        //Yellow case code
        m_ColourToTurnTo = ControlPanelConstants::kGreenTarget;
        break;
      case 'R' :
        //Red case code
        m_ColourToTurnTo = ControlPanelConstants::kBlueTarget;
        break;
      default :
        //This is corrupt data
        // TODO : Throw an error?
        break;
    }

  } 
  else {
    // Code for no data received yet
    // TODO : Throw an error?
  }
}