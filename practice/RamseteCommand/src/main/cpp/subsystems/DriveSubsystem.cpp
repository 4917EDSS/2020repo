/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_leftMotor1{CanIds::kLeftMotor1CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_leftMotor2{CanIds::kLeftMotor2CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_leftMotor3{CanIds::kLeftMotor3CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_rightMotor1{CanIds::kRightMotor1CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_rightMotor2{CanIds::kRightMotor2CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_rightMotor3{CanIds::kRightMotor3CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_leftEncoder{m_leftMotor1.GetEncoder()},
      m_rightEncoder{m_rightMotor1.GetEncoder()},
      m_gyro{frc::SPI::kMXP},
      m_odometry{frc::Rotation2d(units::degree_t(GetHeading()))} {

  // TODO:  Do we need encoders to be on the output shaft vs on the motor?
  
  // Set the distance per pulse for the encoders
  // WARNING!  This value can get erased during brownouts.  Safer to do the conversion in the roboRIO TODO.
  m_leftEncoder.SetPositionConversionFactor(kEncoderDistancePerPulse);
  m_rightEncoder.SetPositionConversionFactor(kEncoderDistancePerPulse);

  ResetEncoders();
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(frc::Rotation2d(units::degree_t(GetHeading())),
                    units::meter_t(m_leftEncoder.GetPosition()),
                    units::meter_t(m_rightEncoder.GetPosition()));
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMotors.SetVoltage(left);
  m_rightMotors.SetVoltage(-right);
}

void DriveSubsystem::ResetEncoders() {
  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
}

double DriveSubsystem::GetAverageEncoderDistance() {
  return (m_leftEncoder.GetPosition() + m_rightEncoder.GetPosition()) / 2.0;
}

// TODO:  Do we need this?  Problem with SparkMax encoder
//frc::Encoder& DriveSubsystem::GetLeftEncoder() { return m_leftEncoder; }

//frc::Encoder& DriveSubsystem::GetRightEncoder() { return m_rightEncoder; }

void DriveSubsystem::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

double DriveSubsystem::GetHeading() {
  return std::remainder(m_gyro.GetAngle(), 360) * (kGyroReversed ? -1.0 : 1.0);
}

double DriveSubsystem::GetTurnRate() {
  return m_gyro.GetRate() * (kGyroReversed ? -1.0 : 1.0);
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds() {
  return {units::meters_per_second_t(m_leftEncoder.GetVelocity()),
          units::meters_per_second_t(m_rightEncoder.GetVelocity())};
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(pose,
                           frc::Rotation2d(units::degree_t(GetHeading())));
}
