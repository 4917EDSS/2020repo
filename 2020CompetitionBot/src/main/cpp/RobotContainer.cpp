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
#include "commands/DisableAutoShiftCmd.h"
#include "commands/AimShootGrp.h"
#include "commands/IntakeCmd.h"
#include "commands/SetHoodPositionCmd.h"
#include "commands/SetHoodSpeedCmd.h"
#include "Constants.h"
#include "subsystems/ClimberSub.h"
#include "commands/ClimbReleaseCmd.h"
#include "commands/ClimbWinchCmd.h"
#include "commands/VisionAlignmentCmd.h"
#include "commands/ClimbBalanceCmd.h"
#include "commands/TurnControlPanelThreeTimesCmd.h"
#include "commands/FlipUpCtrlPanelArmCmd.h"
#include "commands/TurnControlPanelToColourCmd.h"
#include "subsystems/VisionSub.h"
#include "commands/KillEverythingCmd.h"
#include "commands/RamseteCmd.h"


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
constexpr int kClimbWinchReleaseBtn=1;
constexpr int kIntakeBtn=2;
constexpr int kConrolPanelUnfoldBtn=4;
constexpr int kShooterCloseBtn=7;
constexpr int kShooterFarBtn=8;
constexpr int kTurnControlPanelToColourBtn=9;
constexpr int kTurnControlPanelThreeTimesBtn=10;
constexpr int kKillEverything1Btn = 11;
constexpr int kKillEverything2Btn = 12;


//Driver Buttons
constexpr int kDisableAutoShiftBtn=6;
constexpr int kClimbBalanceLeftBtn=7;
constexpr int kClimbBalanceRightBtn=8;
//Going to integrate the alignment with shooterBtn, just hasn't happened yet
constexpr int kShortCameraAlignmentBtn=9;
constexpr int kFarCameraAlignmentBtn=10;
// See driver
// constexpr int kKillEverything1Btn = 11;
// constexpr int kKillEverything2Btn = 12;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  //m_autonomousCommand(&m_shooterSub);
  // Configure the button bindings
  configureButtonBindings();
  autoChooserSetup();
  frc::SmartDashboard::PutData("Set Hood High", new SetHoodPositionCmd(&m_shooterSub, 0.0));
  frc::SmartDashboard::PutData("Set Hood Low", new SetHoodPositionCmd(&m_shooterSub, -14000));
  frc::SmartDashboard::PutNumber("flywheelSpeed", 0.0);
  m_drivetrainSub.SetDefaultCommand(DriveWithJoystickCmd(&m_drivetrainSub, &m_driverController));
  m_shooterSub.SetDefaultCommand(SetHoodSpeedCmd(&m_shooterSub, &m_operatorController));
  // m_shooterSub.SetDefaultCommand(frc2::RunCommand(
  //   [this] {
  //     m_shooterSub.setSpeed(frc::SmartDashboard::GetNumber("flywheelSpeed", 0.0));
  //   },
  // {&m_shooterSub}));
  m_climberSub.SetDefaultCommand(ClimbWinchCmd(&m_climberSub, &m_operatorController));
}

void RobotContainer::initSubsystems() {
  m_climberSub.init();
  m_controlPanelSub.init();
  m_drivetrainSub.init();
  m_intakeSub.init();
  m_shooterSub.init();
  m_visionSub.init();
}

void RobotContainer::autoChooserSetup() {
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
    frc::SimpleMotorFeedforward<units::meters> (DriveConstants::ks, DriveConstants::kv, DriveConstants:: ka),
    DriveConstants::kDriveKinematics, 
    10_V);

  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
  config.SetKinematics(DriveConstants::kDriveKinematics);
  config.AddConstraint(autoVoltageConstraint);

  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
    {},
    frc::Pose2d(1_m, 0_m, frc::Rotation2d(0_deg)),
    config);

  m_autoChooser.AddOption("Ramsete", new RamseteCmd(exampleTrajectory, &m_drivetrainSub));
  m_autoChooser.SetDefaultOption("IntakeCmd", new IntakeCmd(&m_intakeSub));

  frc::SmartDashboard::PutData("Auto Chooser", &m_autoChooser);
}

void RobotContainer::configureButtonBindings() {
  // Configure your button bindings here

  //Driver Commands...

   frc2::JoystickButton disableAutoShiftBtn(&m_driverController, kDisableAutoShiftBtn);
  disableAutoShiftBtn.WhenHeld(DisableAutoShiftCmd(&m_drivetrainSub));

  frc2::JoystickButton farCameraAlignmentBtn(&m_driverController, kFarCameraAlignmentBtn);
  farCameraAlignmentBtn.WhenPressed(VisionAlignmentCmd(&m_visionSub, &m_drivetrainSub, true));

  frc2::JoystickButton shortCameraAlignmentBtn(&m_driverController, kShortCameraAlignmentBtn);
  shortCameraAlignmentBtn.WhenPressed(VisionAlignmentCmd(&m_visionSub, &m_drivetrainSub, false));

  frc2::JoystickButton killEverythingBtn1d(&m_driverController, kKillEverything1Btn);
  killEverythingBtn1d.WhenPressed(KillEverythingCmd(&m_climberSub, &m_controlPanelSub, &m_drivetrainSub, &m_intakeSub, &m_shooterSub, &m_visionSub));

  frc2::JoystickButton killEverythingBtn2d(&m_driverController, kKillEverything2Btn);
  killEverythingBtn2d.WhenPressed(KillEverythingCmd(&m_climberSub, &m_controlPanelSub, &m_drivetrainSub, &m_intakeSub, &m_shooterSub, &m_visionSub));

  frc2::JoystickButton climbBalanceRightBtn(&m_driverController, kClimbBalanceRightBtn);
  climbBalanceRightBtn.WhenHeld(ClimbBalanceCmd(&m_climberSub, true));

  frc2::JoystickButton climbBalanceLeftBtn(&m_driverController, kClimbBalanceLeftBtn);
  climbBalanceLeftBtn.WhenHeld(ClimbBalanceCmd(&m_climberSub, false));


  //Operator Commands...

  frc2::JoystickButton shooterFarBtn(&m_operatorController, kShooterFarBtn);
  shooterFarBtn.WhenHeld(AimShootGrp(&m_visionSub, &m_drivetrainSub, true, &m_shooterSub, &m_intakeSub));

  frc2::JoystickButton shooterCloseBtn(&m_operatorController, kShooterCloseBtn);
  shooterCloseBtn.WhenHeld(AimShootGrp(&m_visionSub, &m_drivetrainSub, false, &m_shooterSub, &m_intakeSub));

  frc2::JoystickButton intakeBtn(&m_operatorController, kIntakeBtn);
  intakeBtn.WhenHeld(IntakeCmd(&m_intakeSub));

 frc2::JoystickButton climbWinchReleaseBtn(&m_operatorController, kClimbWinchReleaseBtn);
  climbWinchReleaseBtn.WhenPressed(ClimbReleaseCmd(&m_climberSub, &m_operatorController));

  frc2::JoystickButton turnControlPanelThreeTimesBtn(&m_operatorController, kTurnControlPanelThreeTimesBtn);
  turnControlPanelThreeTimesBtn.WhenPressed(frc2::SequentialCommandGroup{FlipUpCtrlPanelArmCmd(&m_controlPanelSub), TurnControlPanelThreeTimesCmd(&m_controlPanelSub)});

  frc2::JoystickButton turnControlPanelToColourBtn(&m_operatorController, kTurnControlPanelToColourBtn);
  turnControlPanelToColourBtn.WhenPressed(TurnControlPanelToColourCmd(&m_controlPanelSub));

  frc2::JoystickButton killEverythingBtn1o(&m_operatorController, kKillEverything1Btn);
  killEverythingBtn1o.WhenPressed(KillEverythingCmd(&m_climberSub, &m_controlPanelSub, &m_drivetrainSub, &m_intakeSub, &m_shooterSub, &m_visionSub));

  frc2::JoystickButton killEverythingBtn2o(&m_operatorController, kKillEverything2Btn);
  killEverythingBtn2o.WhenPressed(KillEverythingCmd(&m_climberSub, &m_controlPanelSub, &m_drivetrainSub, &m_intakeSub, &m_shooterSub, &m_visionSub));

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
  return m_autoChooser.GetSelected();
}