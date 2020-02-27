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
#include <frc/util/color.h>

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
    constexpr int kTopIntakeMotor = 11;
    constexpr int kClimbBalanceMotor = 9;
    constexpr int kElevatorMotor = 13;
    constexpr int kControlPanelMotor = 14;
    constexpr int kFrontRollerIntakeMotor = 16;
    constexpr int kShootMotor1 = 25;
    constexpr int kShootMotor2 = 26;
    constexpr int kFeederMotor = 27;
    constexpr int kHoodMotor=20;
}

namespace PneumaticIds {
    //constexpr int Id = 0;
    constexpr int kShifterId = 1;
    constexpr int kControlPanelFlipper = 2;
    constexpr int kClimbReleaseLatch = 3;
}

namespace DioIds {
    constexpr int kFrontIntakeSensorL = 0;
    constexpr int kFrontIntakeSensorR = 1;
    constexpr int kMagazineFullSensor = 2;
}

namespace DriveConstants {
    // constexpr bool kLeftEncoderReversed = false;
    // constexpr bool RightEncoderReversed = true;

    constexpr auto kTrackwidth = 0.67_m; //this is the human measured constant
    //this is the characterized number: 0.797108131
    extern const frc::DifferentialDriveKinematics kDriveKinematics;

    // constexpr int kEncoderCPR = 1024;
    // constexpr double kWheelDiameterInches = 8;
    //  // Assumes the encoders are directly mounted on the wheel shafts
    // constexpr double kEncoderDistancePerPulse =
    //   (kWheelDiameterInches * wpi::math::pi) / static_cast<double>(kEncoderCPR);
    constexpr int kSmartCurrentLimit = 30;

    constexpr bool kGyroReversed = true;

    constexpr auto ks = 0.102_V;
    constexpr auto kv = 3.95 * 1_V * 1_s / 1_m;
    constexpr auto ka = 0.428 * 1_V * 1_s * 1_s / 1_m;

    constexpr double kPDriveVel = 14.3;
}  // namespace DriveConstants

namespace AutoConstants {
    constexpr auto kMaxSpeed = 2.93_mps;
    constexpr auto kMaxAcceleration = 7_mps_sq;

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
    constexpr double kClimbWinchPower = 1.0;
    constexpr double kMoveOnGenSwitchPower = 10.0;
}

namespace ControlPanelConstants {
    constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
    constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
    constexpr frc::Color kRedTarget = frc::Color(0.473, 0.369, 0.154);
    constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
    constexpr double kMaxWheelSpeed = 0.2;
}

namespace ShooterConstants {
    constexpr double kHighHood = 0.0;
    constexpr double kLowHood = 19152.0;
    constexpr double kMaxRPM = 21750;
    constexpr double kCloseTargetSpeed = 12000;
    constexpr double kFarTargetSpeed = 14000;   
}
