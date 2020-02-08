/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <units/units.h>
#include <wpi/math>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace CanIds {
    constexpr int kRightMotor1 = 1;
    constexpr int kRightMotor2 = 2;
    constexpr int kRightMotor3 = 3;
    constexpr int kRightMotor4 = 4;
    constexpr int kLeftMotor1 = 5;
    constexpr int kLeftMotor2 = 6;
    constexpr int kLeftMotor3 = 7;
    constexpr int kLeftMotor4 = 8;
    constexpr int kBottomIntakeMotor = 9;
    constexpr int kTopIntakeMotor = 10;
    constexpr int kClimbBalanceMotor = 11;
    constexpr int kElevatorMotor = 13;
    constexpr int kControlPanelMotor = 14;
    constexpr int kFrontRollerIntakeMotor = 15;
    constexpr int kShootMotor1 = 25;
    constexpr int kShootMotor2 = 26;
    constexpr int kFeederMotor = 27;
}

namespace PneumaticIds {
    //constexpr int Id = 0;
    constexpr int kShifterId = 1;
    constexpr int kControlPanelFlipper = 2;
    constexpr int kClimbReleaseLatch = 3;
}

namespace DioIds {
    constexpr int kFrontIntakeSensor = 1;
    constexpr int kMagazineFullSensor = 2;
}

namespace DriveConstants {
    constexpr bool kLeftEncoderReversed = false;
    constexpr bool RightEncoderReversed = true;

    constexpr auto kTrackwidth = 0.69_m;
    extern const frc::DifferentialDriveKinematics kDriveKinematics;

    constexpr int kEncoderCPR = 1024;
    constexpr double kWheelDiameterInches = 8;
     // Assumes the encoders are directly mounted on the wheel shafts
    constexpr double kEncoderDistancePerPulse =
      (kWheelDiameterInches * wpi::math::pi) / static_cast<double>(kEncoderCPR);
    constexpr int kSmartCurrentLimit = 30;

    constexpr bool kGyroReversed = true;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for *your* robot's drive. The Robot Characterization
    // Toolsuite provides a convenient tool for obtaining these values for your
    // robot.
    constexpr auto ks = 0.22_V;
    constexpr auto kv = 1.98 * 1_V * 1_s / 1_m;
    constexpr auto ka = 0.2 * 1_V * 1_s * 1_s / 1_m;

    // Example value only - as above, this must be tuned for your drive!
    constexpr double kPDriveVel = 8.5;
}  // namespace DriveConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;

// Reasonable baseline values for a RAMSETE follower in units of meters and
// seconds
constexpr double kRamseteB = 2;
constexpr double kRamseteZeta = 0.7;
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 1;
}  // namespace OIConstants

namespace VisionConstants {
    constexpr double kXAllignmentTolerence = 0.5;
    constexpr double kXMax = 30.0;
    constexpr double kFrontCameraId = 1;
}  // namespace VisionConstants

namespace ClimbConstants {
    constexpr double kMoveOnGenSwitchPower = 10.0;
}
