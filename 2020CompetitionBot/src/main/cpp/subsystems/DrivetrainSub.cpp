/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DrivetrainSub.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

constexpr units::velocity::meters_per_second_t kShiftUpSpeed = 3.0_mps;
constexpr units::velocity::meters_per_second_t kShiftDownSpeed = 1.0_mps;

using namespace DriveConstants;

DrivetrainSub::DrivetrainSub() 
    : m_leftMotor1{CanIds::kLeftMotor1CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_leftMotor2{CanIds::kLeftMotor2CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_leftMotor3{CanIds::kLeftMotor3CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_leftMotor4{CanIds::kLeftMotor4CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_rightMotor1{CanIds::kRightMotor1CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_rightMotor2{CanIds::kRightMotor2CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_rightMotor3{CanIds::kRightMotor3CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_rightMotor4{CanIds::kRightMotor4CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_leftEncoder{m_leftMotor1.GetEncoder()},
      m_rightEncoder{m_rightMotor1.GetEncoder()},
      m_gyro{frc::SPI::kMXP},
      m_odometry{frc::Rotation2d(units::degree_t(GetHeading()))},
      m_shifter{PneumaticIds::kShifterId} {

// TODO: Do we need encoder to be on the output shaft vs on the motor?
// Set the distance per pulse for the encoders
// WARNING!  This value can get erased during brownouts.  Safer to do the conversion in the roboRIO TODO.
    m_leftEncoder.SetPositionConversionFactor(kEncoderDistancePerPulse);
    m_rightEncoder.SetPositionConversionFactor(kEncoderDistancePerPulse);

    ResetEncoders();
    m_shifter.Set(false);

    frc::SmartDashboard::PutNumber("drive power", 0);
}

// This method will be called once per scheduler run
void DrivetrainSub::Periodic() {

  m_odometry.Update(frc::Rotation2d(units::degree_t(GetHeading())),
                    units::meter_t(m_leftEncoder.GetPosition()),
                    units::meter_t(m_rightEncoder.GetPosition()));

  // Hack to test motors since joysticks aren't coded
  double currentPower = frc::SmartDashboard::GetNumber("drive power", 0);
  //ArcadeDrive(currentPower, 0);  

  frc::DifferentialDriveWheelSpeeds wheelSpeeds = GetWheelSpeeds();
  units::velocity::meters_per_second_t averageSpeed = (wheelSpeeds.left + wheelSpeeds.right) / 2;
  if(averageSpeed > kShiftUpSpeed) {
    m_shifter.Set(true);
  }
  else if(averageSpeed < kShiftDownSpeed) {
    m_shifter.Set(false);
  }
}

void DrivetrainSub::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
}

void DrivetrainSub::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMotors.SetVoltage(left);
  m_rightMotors.SetVoltage(-right);
}

void DrivetrainSub::ResetEncoders() {
  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
}

double DrivetrainSub::GetAverageEncoderDistance() {
  return (m_leftEncoder.GetPosition() + m_rightEncoder.GetPosition()) / 2.0;
}

void DrivetrainSub::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

double DrivetrainSub::GetHeading() {
  return std::remainder(m_gyro.GetAngle(), 360) * (kGyroReversed ? -1.0 : 1.0);
}

double DrivetrainSub::GetTurnRate() {
  return m_gyro.GetRate() * (kGyroReversed ? -1.0 : 1.0);
}

frc::Pose2d DrivetrainSub::GetPose() { return m_odometry.GetPose(); }

frc::DifferentialDriveWheelSpeeds DrivetrainSub::GetWheelSpeeds() {
  return {units::meters_per_second_t(m_leftEncoder.GetVelocity()),
          units::meters_per_second_t(m_rightEncoder.GetVelocity())};
}

void DrivetrainSub::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(pose,
                           frc::Rotation2d(units::degree_t(GetHeading())));
}
