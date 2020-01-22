/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"
#include <frc2/command/button/JoystickButton.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include "commands/ShootCmd.h"
#include "commands/IntakeCmd.h"
#include "Constants.h"


constexpr int SHOOTER_BTN=2;
constexpr int INTAKE_BTN=1;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  //m_autonomousCommand(&m_ShooterSub);
  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  frc2::JoystickButton m_shooterBtn(&m_driverController, SHOOTER_BTN);
  m_shooterBtn.WhenPressed(ShootCmd(&m_ShooterSub,&m_IntakeSub));

  frc2::JoystickButton m_intakeBtn(&m_operatorController, INTAKE_BTN);
  m_intakeBtn.WhenHeld(IntakeCmd(&m_IntakeSub));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_autonomousCommand;
}
