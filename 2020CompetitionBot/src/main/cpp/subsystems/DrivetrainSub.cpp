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
#include "RobotContainer.h"

constexpr float kEncoderTicksToMm = 30.928;
constexpr units::velocity::meters_per_second_t kShiftUpSpeed = 3.0_mps;
constexpr units::velocity::meters_per_second_t kShiftDownSpeed = 1.0_mps;

DrivetrainSub::DrivetrainSub() 
  : m_leftMotor1{CanIds::kLeftMotor1CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_leftMotor2{CanIds::kLeftMotor2CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_leftMotor3{CanIds::kLeftMotor3CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_leftMotor4{CanIds::kLeftMotor4CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_rightMotor1{CanIds::kRightMotor1CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_rightMotor2{CanIds::kRightMotor2CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_rightMotor3{CanIds::kRightMotor3CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_rightMotor4{CanIds::kRightMotor4CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_gyro{frc::SPI::kMXP},
    m_odometry{frc::Rotation2d(units::degree_t(getHeading()))},
    m_shifter{PneumaticIds::kShifterId},
    m_isAutoShiftEnabled{false} {

// TODO: Do we need encoder to be on the output shaft vs on the motor?
// Set the distance per pulse for the encoders
// WARNING!  This value can get erased during brownouts.  Safer to do the conversion in the getLeftEncoder and getRightEncoder functions TODO.
  m_shifter.Set(false);
  setDrivetrainEncoderZero();

  m_rightMotor1.SetSmartCurrentLimit(50);
  m_rightMotor2.SetSmartCurrentLimit(50);
  m_rightMotor3.SetSmartCurrentLimit(50);
  m_rightMotor4.SetSmartCurrentLimit(50);

  m_leftMotor1.SetSmartCurrentLimit(50);
  m_leftMotor2.SetSmartCurrentLimit(50);
  m_leftMotor3.SetSmartCurrentLimit(50);
  m_leftMotor4.SetSmartCurrentLimit(50);


  frc::SmartDashboard::PutNumber("drive power", 0);
  frc::SmartDashboard::PutBoolean("autoshifter", m_isAutoShiftEnabled);

  arcadeDrive(0.2, 0);
}

// This method will be called once per scheduler run
void DrivetrainSub::Periodic() {

  m_odometry.Update(frc::Rotation2d(units::degree_t(getHeading())),
                    units::meter_t(getLeftEncoder()),
                    units::meter_t(getRightEncoder()));

  m_isAutoShiftEnabled = frc::SmartDashboard::GetBoolean("autoshifter", m_isAutoShiftEnabled);
  if(m_isAutoShiftEnabled) {
    autoShift();
  }

  m_drive.Feed();
}

void DrivetrainSub::setDrivetrainEncoderZero(){
  m_rightMotor1.GetEncoder().SetPosition(0);
  m_rightMotor2.GetEncoder().SetPosition(0);
  m_rightMotor3.GetEncoder().SetPosition(0);
  m_rightMotor4.GetEncoder().SetPosition(0);

  m_leftMotor1.GetEncoder().SetPosition(0);
  m_leftMotor2.GetEncoder().SetPosition(0);
  m_leftMotor3.GetEncoder().SetPosition(0);
  m_leftMotor4.GetEncoder().SetPosition(0);
}

void DrivetrainSub::arcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
  printf("fwd=%4.2f rot=%4.2f\n", fwd, rot);
}

void DrivetrainSub::shiftUp(){
        m_shifter.Set(true);
  }

 void DrivetrainSub::shiftDown(){
        m_shifter.Set(false);
  }

void DrivetrainSub::tankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMotors.SetVoltage(left);
  m_rightMotors.SetVoltage(-right);
}

double DrivetrainSub::getAverageEncoderDistance() {
  return (getLeftEncoder() + getRightEncoder()) / 2.0;
}

void DrivetrainSub::setMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

double DrivetrainSub::getHeading() {
  return std::remainder(m_gyro.GetAngle(), 360) * (DriveConstants::kGyroReversed ? -1.0 : 1.0);
}

double DrivetrainSub::getTurnRate() {
  return m_gyro.GetRate() * (DriveConstants::kGyroReversed ? -1.0 : 1.0);
}

frc::Pose2d DrivetrainSub::getPose() { return m_odometry.GetPose(); }

frc::DifferentialDriveWheelSpeeds DrivetrainSub::getWheelSpeeds() {
  return {units::meters_per_second_t(getLeftEncoder()),
          units::meters_per_second_t(getRightEncoder())};
}

void DrivetrainSub::resetOdometry(frc::Pose2d pose) {
  setDrivetrainEncoderZero();
  m_odometry.ResetPosition(pose,
                           frc::Rotation2d(units::degree_t(getHeading())));
}

double DrivetrainSub::getRightEncoder()
{
  return (m_rightMotor1.GetEncoder().GetPosition()*kEncoderTicksToMm);
}

double DrivetrainSub::getLeftEncoder()
{
    return (m_leftMotor1.GetEncoder().GetPosition()*kEncoderTicksToMm);
}

void DrivetrainSub::autoShift() {
  frc::DifferentialDriveWheelSpeeds wheelSpeeds = getWheelSpeeds();
  units::velocity::meters_per_second_t averageSpeed = (wheelSpeeds.left + wheelSpeeds.right) / 2;
  if(averageSpeed > kShiftUpSpeed) {
    m_shifter.Set(true);
  }
  else if(averageSpeed < kShiftDownSpeed) {
    m_shifter.Set(false);
  }
}