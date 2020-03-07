/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ManualControlPanel.h"

ManualControlPanel::ManualControlPanel(ControlPanelSub* controlPanelSub, frc::Joystick* joystick) 
: m_controlPanelSub(controlPanelSub),
  m_joystick(joystick) {
  // Use addRequirements() here to declare subsystem dependencies.
  

}

// Called when the command is initially scheduled.
void ManualControlPanel::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void ManualControlPanel::Execute() {
  double wheelPower = m_joystick->GetY();
   m_controlPanelSub->setWheelPower(wheelPower);
}

// Called once the command ends or is interrupted.
void ManualControlPanel::End(bool interrupted) {
   m_controlPanelSub->setWheelPower(0);
}

// Returns true when the command should end.
bool ManualControlPanel::IsFinished() { return false; }
