/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/FlipUpCtrlPanelArmCmd.h"
#include <frc/RobotController.h>


FlipUpCtrlPanelArmCmd::FlipUpCtrlPanelArmCmd(ControlPanelSub* controlPanelSub): m_controlPanelSub(controlPanelSub) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({controlPanelSub});
}

// Called when the command is initially scheduled.
void FlipUpCtrlPanelArmCmd::Initialize() {
  m_controlPanelSub->togglePosition(true);
   m_initializedTime = frc::RobotController::GetFPGATime();
  

}

// Called repeatedly when this Command is scheduled to run
void FlipUpCtrlPanelArmCmd::Execute() {
}

// Called once the command ends or is interrupted.
void FlipUpCtrlPanelArmCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool FlipUpCtrlPanelArmCmd::IsFinished() { 
  if ( (frc::RobotController::GetFPGATime() - m_initializedTime) > 500000) {
    return true; 
  }
  else {
    return false;
  }
}
