/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc/Joystick.h>

#include "commands/ExampleCommand.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/DrivetrainSub.h"
#include "subsystems/ColorSub.h"

constexpr int DRIVER_JOYSTICK_PORT = 0;

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  // The robot's subsystems and commands are defined here...
  ExampleSubsystem m_subsystem;
  DrivetrainSub m_drivetrainSub;
  
  ColorSub m_colorSub;

  
  ExampleCommand m_autonomousCommand;

  // Controllers and buttons
  frc::Joystick m_driverController{DRIVER_JOYSTICK_PORT}; 
  
  // Special commands
  frc2::InstantCommand m_auxMotorSlowIcmd{[this] { m_drivetrainSub.setAuxPower(0.1); }, {&m_drivetrainSub}};
  frc2::InstantCommand m_auxMotorOffIcmd{[this] { m_drivetrainSub.setAuxPower(0.0); }, {&m_drivetrainSub}};
  
  void ConfigureButtonBindings();
};
