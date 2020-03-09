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
    constexpr int kControlPanelMotor = 15;
    constexpr int kFrontRollerIntakeMotor = 16;
    constexpr int kShootMotor1 = 25;
    constexpr int kShootMotor2 = 26;
    constexpr int kFeederMotor = 27;
    constexpr int kHoodMotor=20;
}

namespace PneumaticIds {
    //constexpr int Id = 0;
    constexpr int kShifterId = 1;
    constexpr int kClimbReleaseLatch = 2;
    constexpr int kControlPanelFlipper = 3;
    constexpr int kHoodAdjuster = 4;
}

namespace DioIds {
    constexpr int kFrontIntakeSensorL = 0;
    constexpr int kFrontIntakeSensorR = 1;
    constexpr int kMagazineFrontSensorL = 2;
    constexpr int kMagazineFrontSensorR = 3;
    constexpr int kMagazineFullSensor = 4;
}

namespace DpadConstants {
    constexpr int kInactive = -1;
    constexpr int kUp = 0;
    constexpr int kUpRight = 45;
    constexpr int kRight = 90;
    constexpr int kRightDown = 135;
    constexpr int kDown = 180;
    constexpr int kDownLeft = 225;
    constexpr int kLeft = 270;
    constexpr int kLeftUp = 315;
}