/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/PrintCommand.h>

#include "RobotContainer.h"

#include "commands/DriveWithJoystickCmd.h"
#include "commands/AuxMotorReverseCmd.h"

constexpr int AUX_MOTOR_SLOW_BTN = 1;
constexpr int AUX_MOTOR_OFF_BTN = 2;
constexpr int PRINT_MSG_BTN = 3;
constexpr int AUX_REVERSE_TIMED_BTN = 4;

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here
  frc::SmartDashboard::PutData("Drivetrain", &m_drivetrainSub);
  frc::SmartDashboard::PutData("AuxSlowCmd", &m_auxMotorSlowIcmd);
  frc::SmartDashboard::PutData("AuxOffCmd", &m_auxMotorOffIcmd);
  frc::SmartDashboard::PutData("AuxReverseCmd", new AuxMotorReverseCmd(&m_drivetrainSub, -0.1, 5));
 
  // Configure the button bindings
  ConfigureButtonBindings();

  // Set default commands
  m_drivetrainSub.SetDefaultCommand(DriveWithJoystickCmd(&m_drivetrainSub, &m_driverController));
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  //frc2::JoystickButton m_auxMotorSlowBtn(&m_driverController, AUX_MOTOR_SLOW_BTN);
  //frc2::JoystickButton m_auxMotorOffBtn(&m_driverController, AUX_MOTOR_OFF_BTN);
  frc2::JoystickButton m_printMsgBtn(&m_driverController, PRINT_MSG_BTN);
  //frc2::JoystickButton m_auxReverseTimedBtn(&m_driverController, AUX_REVERSE_TIMED_BTN);
  
  //m_auxMotorSlowBtn.WhenPressed(&m_auxMotorSlowIcmd);
  //m_auxMotorOffBtn.WhenPressed(&m_auxMotorOffIcmd);
  m_printMsgBtn.WhenPressed(frc2::PrintCommand("Print message button pressed"));
  //m_auxReverseTimedBtn.WhenPressed(AuxMotorReverseCmd(&m_drivetrainSub, -0.1, 5));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
