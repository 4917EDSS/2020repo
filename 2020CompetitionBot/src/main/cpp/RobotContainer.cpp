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
#include "commands/SetHoodPositionCmd.h"
#include "Constants.h"
#include "subsystems/ClimberSub.h"
#include "commands/ClimbReleaseCmd.h"
#include "commands/ClimbWinchCmd.h"
#include "commands/VisionAlignmentCmd.h"
#include "commands/ClimbBalanceCmd.h"
#include "commands/TurnControlPanelThreeTimesCmd.h"
#include "commands/FlipUpCtrlPanelArmCmd.h"
#include "subsystems/VisionSub.h"


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
constexpr int kTurnControlPanelThreeTimes=5;
constexpr int kTurnControlPahnelToColour=6;

//Driver Buttons
constexpr int kShiftUpBtn=5;
constexpr int kShiftDownBtn=6;
constexpr int kClimbBalanceLeft=7;
constexpr int kClimbBalanceRight=8;
constexpr int kFarCameraAlignment=9;
constexpr int kShortCameraAlignment=10;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  //m_autonomousCommand(&m_shooterSub);
  // Configure the button bindings
  configureButtonBindings();
  autoChooserSetup();
  frc::SmartDashboard::PutData("Set Hood none", new SetHoodPositionCmd(&m_shooterSub,0.0));
  frc::SmartDashboard::PutData("Set Hood Medium", new SetHoodPositionCmd(&m_shooterSub,12000.0));
  frc::SmartDashboard::PutData("Set Hood High", new SetHoodPositionCmd(&m_shooterSub,24000.0));
  frc::SmartDashboard::PutNumber("flywheelSpeed", 0.0);
  m_drivetrainSub.SetDefaultCommand(DriveWithJoystickCmd(&m_drivetrainSub, &m_driverController));
  m_shooterSub.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_shooterSub.setSpeed(frc::SmartDashboard::GetNumber("flywheelSpeed", 0.0));
    },
  {&m_shooterSub}));
}

void RobotContainer::generateTrajectories() {
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
    frc::SimpleMotorFeedforward < units::meters > (
      DriveConstants::ks, DriveConstants::kv, DriveConstants:: ka),
        DriveConstants::kDriveKinematics, 10_V);
        frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
        config.SetKinematics(DriveConstants::kDriveKinematics);
        config.AddConstraint(autoVoltageConstraint);
        auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
          {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
          frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
          config);
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

  frc2::JoystickButton farCameraAlignmentBtn(&m_driverController, kFarCameraAlignment);
  farCameraAlignmentBtn.WhenPressed(VisionAlignmentCmd(&m_visionSub, &m_drivetrainSub, true));

  frc2::JoystickButton shortCameraAlignmentBtn(&m_driverController, kShortCameraAlignment);
  shortCameraAlignmentBtn.WhenPressed(VisionAlignmentCmd(&m_visionSub, &m_drivetrainSub, false));

  //Operator Commands...

  frc2::JoystickButton shooterBtn(&m_operatorController, kShooterBtn);
  shooterBtn.WhenHeld(ShootCmd(&m_shooterSub, &m_intakeSub));

  frc2::JoystickButton climbBalanceRightBtn(&m_driverController, kClimbBalanceRight);
  climbBalanceRightBtn.WhenHeld(ClimbBalanceCmd(&m_climberSub, true));

  frc2::JoystickButton climbBalanceLeftBtn(&m_driverController, kClimbBalanceLeft);
  climbBalanceLeftBtn.WhenHeld(ClimbBalanceCmd(&m_climberSub, false));

  frc2::JoystickButton intakeBtn(&m_operatorController, kIntakeBtn);
  intakeBtn.WhenHeld(IntakeCmd(&m_intakeSub));

  frc2::JoystickButton climbReleaseBtn(&m_operatorController, kClimbReleaseBtn);
  climbReleaseBtn.WhenPressed(ClimbReleaseCmd(&m_climberSub));

  frc2::JoystickButton climbWinchBtn(&m_operatorController, kClimbWinchBtn);
  climbWinchBtn.WhenHeld(ClimbWinchCmd(&m_climberSub));

  frc2::JoystickButton turnControlPanelThreeTimesBtn(&m_operatorController, kTurnControlPanelThreeTimes);
  turnControlPanelThreeTimesBtn.WhenPressed(frc2::SequentialCommandGroup{FlipUpCtrlPanelArmCmd(&m_controlPanelSub), TurnControlPanelThreeTimesCmd(&m_controlPanelSub)});

  // frc2::JoystickButton turnControlPanelToColourBtn(&m_operatorController, kTurnControlPanelToColour);
  // turnControlPanelToColourBtn.WhenPressed(TurnControlPanelToColourCmd());

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
