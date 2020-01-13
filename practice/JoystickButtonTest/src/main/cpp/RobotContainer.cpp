/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/PrintCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "commands/PrintHelloCmd.h"

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here
  frc::SmartDashboard::PutData("PrintHelloCmd", new PrintHelloCmd());

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  frc2::JoystickButton m_dUp{&m_joy, 5};
  frc2::JoystickButton m_button1{&m_joy, 1};

  m_dUp.WhenPressed(frc2::PrintCommand("Print message button pressed"));
  m_button1.WhenPressed(PrintHelloCmd());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
