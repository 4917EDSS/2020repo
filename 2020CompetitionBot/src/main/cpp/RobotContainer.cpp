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
#include <frc/smartdashboard/SmartDashboard.h>
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
#include "subsystems/ClimberSub.h"
#include "commands/ClimbReleaseCmd.h"
#include "commands/ClimbWinchCmd.h"

constexpr int SHOOTER_BTN=2;
constexpr int INTAKE_BTN=1;

constexpr int CLIMB_RELEASE_BTN=3;
constexpr int CLIMB_SPOOL_BTN=4;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  //m_autonomousCommand(&m_ShooterSub);
  // Configure the button bindings
  ConfigureButtonBindings();
  AutoChooserSetup();
}

void RobotContainer::AutoChooserSetup(){
  autoChooser.reset(new frc::SendableChooser< std::shared_ptr<frc2::Command>>());

  autoChooser->AddOption("SecondAuto", std::shared_ptr<frc2::Command>(new IntakeCmd(&m_IntakeSub)));
  autoChooser->AddDefault("VictoryLap", std::shared_ptr<frc2::Command>(new IntakeCmd(&m_IntakeSub)));


  frc::SmartDashboard::PutData("Auto Chooser", autoChooser.get());
  frc::SmartDashboard::PutNumber("shooterspeed", 0);
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  frc2::JoystickButton m_shooterBtn(&m_driverController, SHOOTER_BTN);
  m_shooterBtn.WhenPressed(ShootCmd(&m_ShooterSub, &m_IntakeSub, 3000));

  frc2::JoystickButton m_intakeBtn(&m_operatorController, INTAKE_BTN);
  m_intakeBtn.WhenHeld(IntakeCmd(&m_IntakeSub));

  frc2::JoystickButton m_climbReleaseBtn(&m_operatorController, CLIMB_RELEASE_BTN);
  m_climbReleaseBtn.WhenPressed(ClimbReleaseCmd(&m_ClimberSub));

  frc2::JoystickButton m_climbWinchBtn(&m_operatorController, CLIMB_SPOOL_BTN);
  m_climbWinchBtn.WhenHeld(ClimbWinchCmd(&m_ClimberSub));

}



frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous

  return m_autonomousCommand;
}
