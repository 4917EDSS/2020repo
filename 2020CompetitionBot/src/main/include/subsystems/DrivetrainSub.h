/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <frc/Encoder.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <units/units.h>
#include <AHRS.h>
#include <frc/solenoid.h>

#include "Constants.h"

class DrivetrainSub : public frc2::SubsystemBase {
 public:
  DrivetrainSub();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void setDrivetrainEncoderZero();
  void drive(double lSpeed, double rSpeed);
  void autoShift();
  double getLeftEncoder();
  double getRightEncoder();
  // Subsystem methods go here.

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  void arcadeDrive(double fwd, double rot);

/**
   * Controls each side of the drive directly with a voltage.
   *
   * @param left the commanded left output
   * @param right the commanded right output
   */
  void tankDriveVolts(units::volt_t left, units::volt_t right);
  /**
   * Resets the drive encoders to currently read a position of 0.
   */
 void ResetEncoders();

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  double getAverageEncoderDistance();

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
 // frc::Encoder& GetLeftEncoder();

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
 // frc::Encoder& GetRightEncoder();

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive
   * more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  void setMaxOutput(double maxOutput);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  double getHeading();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double getTurnRate();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d getPose();

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  frc::DifferentialDriveWheelSpeeds getWheelSpeeds();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void resetOdometry(frc::Pose2d pose);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  rev::CANSparkMax m_leftMotor1;
  rev::CANSparkMax m_leftMotor2;
  rev::CANSparkMax m_leftMotor3;
  rev::CANSparkMax m_leftMotor4;
  rev::CANSparkMax m_rightMotor1;
  rev::CANSparkMax m_rightMotor2;
  rev::CANSparkMax m_rightMotor3;
  rev::CANSparkMax m_rightMotor4;

  // The motors on the left side of the drive
  frc::SpeedControllerGroup m_leftMotors{m_leftMotor1, m_leftMotor2, m_leftMotor3, m_leftMotor4};

  // The motors on the right side of the drive
  frc::SpeedControllerGroup m_rightMotors{m_rightMotor1, m_rightMotor2, m_rightMotor3, m_rightMotor4};

  // The robot's drive
  frc::DifferentialDrive m_drive{m_leftMotors, m_rightMotors};

  // The gyro sensor
  AHRS m_gyro;

  // Odometry class for tracking robot pose
  frc::DifferentialDriveOdometry m_odometry;

  frc::Solenoid m_shifter;
};
