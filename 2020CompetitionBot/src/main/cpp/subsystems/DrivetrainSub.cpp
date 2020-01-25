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

constexpr units::velocity::meters_per_second_t kShiftUpSpeed = 3.0_mps;
constexpr units::velocity::meters_per_second_t kShiftDownSpeed = 1.0_mps;

using namespace DriveConstants;
constexpr float ENCODER_TICK_TO_MM = 30.928;

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
    m_odometry{frc::Rotation2d(units::degree_t(GetHeading()))},
    m_shifter{PneumaticIds::kShifterId} {

// TODO: Do we need encoder to be on the output shaft vs on the motor?
// Set the distance per pulse for the encoders
// WARNING!  This value can get erased during brownouts.  Safer to do the conversion in the getLeftEncoder and getRightEncoder functions TODO.
  m_shifter.Set(false);
  SetDrivetrainEncoderZero();

  m_rightMotor1.SetSmartCurrentLimit(50);
  m_rightMotor2.SetSmartCurrentLimit(50);
  m_rightMotor3.SetSmartCurrentLimit(50);
  m_rightMotor4.SetSmartCurrentLimit(50);

  m_leftMotor1.SetSmartCurrentLimit(50);
  m_leftMotor2.SetSmartCurrentLimit(50);
  m_leftMotor3.SetSmartCurrentLimit(50);
  m_leftMotor4.SetSmartCurrentLimit(50);


  frc::SmartDashboard::PutNumber("drive power", 0);
}

void DrivetrainSub::SetDrivetrainEncoderZero(){
  m_rightMotor1.GetEncoder().SetPosition(0);
  m_rightMotor2.GetEncoder().SetPosition(0);
  m_rightMotor3.GetEncoder().SetPosition(0);

  m_leftMotor1.GetEncoder().SetPosition(0);
  m_leftMotor2.GetEncoder().SetPosition(0);
  m_leftMotor3.GetEncoder().SetPosition(0);
}

// This method will be called once per scheduler run
void DrivetrainSub::Periodic() {

  m_odometry.Update(frc::Rotation2d(units::degree_t(GetHeading())),
                    units::meter_t(getLeftEncoder()),
                    units::meter_t(getRightEncoder()));

  // Hack to test motors since joysticks aren't coded
  double currentPower = frc::SmartDashboard::GetNumber("drive power", 0);
  //ArcadeDrive(currentPower, 0);  

  autoShift();

  m_drive.Feed();
}

void DrivetrainSub::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
}

void DrivetrainSub::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMotors.SetVoltage(left);
  m_rightMotors.SetVoltage(-right);
}

double DrivetrainSub::GetAverageEncoderDistance() {
  return (getLeftEncoder() + getRightEncoder()) / 2.0;
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
  return {units::meters_per_second_t(getLeftEncoder()),
          units::meters_per_second_t(getRightEncoder())};
}

void DrivetrainSub::ResetOdometry(frc::Pose2d pose) {
  SetDrivetrainEncoderZero();
  m_odometry.ResetPosition(pose,
                           frc::Rotation2d(units::degree_t(GetHeading())));
}

double DrivetrainSub::getRightEncoder()
{
  return (m_rightMotor1.GetEncoder().GetPosition()*ENCODER_TICK_TO_MM);
}

double DrivetrainSub::getLeftEncoder()
{
    return (m_leftMotor1.GetEncoder().GetPosition()*ENCODER_TICK_TO_MM);
}

void DrivetrainSub::autoShift() {
  frc::DifferentialDriveWheelSpeeds wheelSpeeds = GetWheelSpeeds();
  units::velocity::meters_per_second_t averageSpeed = (wheelSpeeds.left + wheelSpeeds.right) / 2;
  if(averageSpeed > kShiftUpSpeed) {
    m_shifter.Set(true);
  }
  else if(averageSpeed < kShiftDownSpeed) {
    m_shifter.Set(false);
  }
}