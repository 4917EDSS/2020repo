/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/Command.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "subsystems/ShooterSub.h"
#include "subsystems/IntakeSub.h"
#include "subsystems/DrivetrainSub.h"
#include "subsystems/ShooterSub.h"
#include "subsystems/ClimberSub.h"
#include "commands/ClimbReleaseCmd.h"
#include "subsystems/VisionSub.h"

constexpr int DRIVER_JOYSTICK_PORT=0;
constexpr int OPERATOR_JOYSTICK_PORT=1;

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

  frc2::Command* getAutonomousCommand();

 private:

  // The robot's subsystems and commands are defined here...
  DrivetrainSub m_drivetrainSub;
  IntakeSub m_intakeSub;
  ShooterSub m_shooterSub;
  ClimberSub m_climberSub;
  frc::SendableChooser<frc2::Command*> m_autoChooser;
  frc2::Command* m_autonomousCommand;
  VisionSub m_visionSub;

  //Controllers and Buttons
  frc::Joystick m_driverController{DRIVER_JOYSTICK_PORT};
  frc::Joystick m_operatorController{OPERATOR_JOYSTICK_PORT};

  void autoChooserSetup();
  void configureButtonBindings();
};
