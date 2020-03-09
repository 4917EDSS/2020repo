/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ManualControlPanelCmd.h"

constexpr double kDeadBand = 0.03;

ManualControlPanelCmd::ManualControlPanelCmd(ControlPanelSub* controlPanelSub, frc::Joystick* joystick) 
   : m_controlPanelSub(controlPanelSub),
     m_joystick(joystick) {

  AddRequirements({controlPanelSub});  
}

// Called when the command is initially scheduled.
void ManualControlPanelCmd::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void ManualControlPanelCmd::Execute() {
  double wheelPower = m_joystick->GetX();
  if(fabs(wheelPower) <= kDeadBand) {
    wheelPower = 0.0;
  }
  m_controlPanelSub->setWheelPower(wheelPower);
}

// Called once the command ends or is interrupted.
void ManualControlPanelCmd::End(bool interrupted) {
  // Joystick-controller commands don't usually have an End
}

// Returns true when the command should end.
bool ManualControlPanelCmd::IsFinished() { return false; }
