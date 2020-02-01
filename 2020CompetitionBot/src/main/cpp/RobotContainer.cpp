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
#include <frc2/command/RunCommand.h>
#include "commands/DriveWithJoystickCmd.h"
#include "commands/ShootCmd.h"
#include "commands/IntakeCmd.h"
#include "Constants.h"
#include "subsystems/ClimberSub.h"
#include "commands/ClimbReleaseCmd.h"
#include "commands/ClimbWinchCmd.h"
#include "commands/VisionAlignmentCmd.h"
#include "commands/ClimbBalanceCmd.h"
/*
 * ON LOGITECH F310 CONTROLLER:
 * X = 1            (Blue)
 * A = 2            (Green)
 * B = 3            (Red)
 * Y = 4            (Yellow)
 * LB = 5           (Left-Bumper: top button)
 * RB = 6           (Right-Bumper: top button)
 * LT = 7           (Left-Trigger: bottom button)
 * RT = 8           (Right-Trigger: bottom button)
 * Select/Back = 9  (Above left joystick)
 * Start = 10       (Above right joytsick)
 * L3 = 11          (Press left joystick)
 * R3 = 12          (Press right joystick)
 * 
 * Left Joystick Vertical Axis = 1
 * Left Joystick Horizontal Axis = 0
 * Right Joystick Vertical Axis = 3
 * Right Joystick Horizontal Axis = 2
 */


//Operator Buttons
constexpr int kIntakeBtn=1;
constexpr int kShooterBtn=2;
constexpr int kClimbReleaseBtn=3;
constexpr int kClimbWinchBtn=4;

//Driver Buttons
constexpr int kShiftUpBtn=5;
constexpr int kShiftDownBtn=6;
constexpr int kClimbBalanceLeft=7;
constexpr int kClimbBalanceRight=8;
constexpr int kFrontCameraAlignment=9;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  //m_autonomousCommand(&m_shooterSub);
  // Configure the button bindings
  configureButtonBindings();
  autoChooserSetup();
m_drivetrainSub.SetDefaultCommand(DriveWithJoystickCmd(&m_drivetrainSub, &m_driverController));
}

void RobotContainer::autoChooserSetup(){
  m_autoChooser.AddOption("SecondAuto", new IntakeCmd(&m_intakeSub));
  m_autoChooser.SetDefaultOption("VictoryLap", new IntakeCmd(&m_intakeSub));


  frc::SmartDashboard::PutData("Auto Chooser", &m_autoChooser);
  frc::SmartDashboard::PutNumber("shooterspeed", 0);
}

void RobotContainer::configureButtonBindings() {
  // Configure your button bindings here

  //Driver Commands...

  frc2::JoystickButton shiftUpBtn(&m_driverController, kShiftUpBtn);
  shiftUpBtn.WhenPressed(frc2::InstantCommand(
  [this] {
    m_drivetrainSub.shiftUp();
        
  },
  {&m_drivetrainSub}));

   frc2::JoystickButton shiftDownBtn(&m_driverController, kShiftDownBtn);
  shiftDownBtn.WhenPressed(frc2::InstantCommand(
  [this] {
    m_drivetrainSub.shiftDown();
        
  },
  {&m_drivetrainSub}));

  //Operator Commands...

  frc2::JoystickButton shooterBtn(&m_operatorController, kShooterBtn);
  shooterBtn.WhenHeld(ShootCmd(&m_shooterSub, &m_intakeSub, 3000));

  frc2::JoystickButton climbBalanceRightBtn(&m_driverController, kClimbBalanceRight);
  climbBalanceRightBtn.WhenHeld(ClimbBalanceCmd(&m_climberSub, true));

  frc2::JoystickButton climbBalanceLeftBtn(&m_driverController, kClimbBalanceLeft);
  climbBalanceLeftBtn.WhenHeld(ClimbBalanceCmd(&m_climberSub, false));

  frc2::JoystickButton frontCameraAlignmentBtn(&m_driverController, kFrontCameraAlignment);
  frontCameraAlignmentBtn.WhenPressed(VisionAlignmentCmd(&m_visionSub, &m_drivetrainSub));

  frc2::JoystickButton intakeBtn(&m_operatorController, kIntakeBtn);
  intakeBtn.WhenHeld(IntakeCmd(&m_intakeSub));

  frc2::JoystickButton climbReleaseBtn(&m_operatorController, kClimbReleaseBtn);
  climbReleaseBtn.WhenPressed(ClimbReleaseCmd(&m_climberSub));

  frc2::JoystickButton climbWinchBtn(&m_operatorController, kClimbWinchBtn);
  climbWinchBtn.WhenHeld(ClimbWinchCmd(&m_climberSub));

  m_driverController.SetXChannel(0);
  m_driverController.SetYChannel(1);
  m_driverController.SetZChannel(2);
  m_driverController.SetThrottleChannel(3);

  m_operatorController.SetXChannel(0);
  m_operatorController.SetYChannel(1);
  m_operatorController.SetZChannel(2);
  m_operatorController.SetThrottleChannel(3);
}

frc2::Command* RobotContainer::getAutonomousCommand() {
  // An example command will be run in autonomous

  return m_autonomousCommand;
}
