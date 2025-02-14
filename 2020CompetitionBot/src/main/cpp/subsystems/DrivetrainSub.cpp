/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>

#include "Constants.h"
#include "subsystems/DrivetrainSub.h"

constexpr float kEncoderRotationsToMLowGear = 5.0/(160.162) / 1.039; // We drove 5.75m but it thought we drove 5.98 so to compensate, divide by by 1.04
constexpr float kEncoderRotationsToMHighGear = 5.0/(102.264);
constexpr double kShiftUpSpeed = 2.35;    // In m/s
constexpr double kShiftDownSpeed = 1.15;  // In m/s
constexpr double kScrubFactor = 0.0;

DrivetrainSub::DrivetrainSub() 
  : m_leftMotor1{CanIds::kLeftMotor1, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_leftMotor2{CanIds::kLeftMotor2, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_leftMotor3{CanIds::kLeftMotor3, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_leftMotor4{CanIds::kLeftMotor4, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_rightMotor1{CanIds::kRightMotor1, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_rightMotor2{CanIds::kRightMotor2, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_rightMotor3{CanIds::kRightMotor3, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_rightMotor4{CanIds::kRightMotor4, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_gyro{frc::SPI::kMXP},
    m_odometry{frc::Rotation2d(units::degree_t(getHeading()))},
    m_shifter{PneumaticIds::kShifterId} {

  m_rightMotor1.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);
  m_rightMotor2.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);
  m_rightMotor3.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);
  m_rightMotor4.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);

  m_leftMotor1.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);
  m_leftMotor2.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);
  m_leftMotor3.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);
  m_leftMotor4.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);

  init();
}

void DrivetrainSub::init() {
  m_gyro.Reset();
  shiftDown();
  setDrivetrainEncoderZero();
  tankDrive(0.0, 0.0);
  m_odometry.ResetPosition(frc::Pose2d(), frc::Rotation2d());
#ifdef RAMSETE_LOG
  std::cout << "RLog," << "FPGATime" << "," << "poseX" << "," << "poseY" << "," << "poseAngle"
            << "," << "wheelSpeedL" << "," << "wheelSpeedR" << "," << "m_wheelVoltageL" << "," << "m_wheelVoltageR"
            << std::endl;
#endif
}

// This method will be called once per scheduler run
void DrivetrainSub::Periodic() {

  double currentHeading = getHeading();
  double angleDiff = fabs(m_lastAngle - currentHeading);
  if (angleDiff > 180) {
   angleDiff = fabs(angleDiff - 360);
  }

  m_odometry.Update(frc::Rotation2d(units::degree_t(currentHeading)),
                    units::meter_t(getLeftEncoderDistanceM()*(1.0 - angleDiff*kScrubFactor)),
                    units::meter_t(getRightEncoderDistanceM()*(1.0 - angleDiff*kScrubFactor)));

  // frc::SmartDashboard::PutNumber("RawEnc R", getRightEncoderRaw());
  // frc::SmartDashboard::PutNumber("RawEnc L", getLeftEncoderRaw());
  // frc::SmartDashboard::PutNumber("CnvrtdEnc R", getRightEncoderDistanceM());
  // frc::SmartDashboard::PutNumber("CnvrtdEnc L", getLeftEncoderDistanceM());
  // frc::SmartDashboard::PutNumber("MtrVlcty R", getRightVelocity());
  // frc::SmartDashboard::PutNumber("MtrVlcty L", getLeftVelocity());
  // frc::SmartDashboard::PutBoolean("High Gear", isShifterInHighGear());
  // std::cout << getPose().Translation().X() << " " << getPose().Translation().Y() << " " << getPose().Rotation().Radians() << "\n";
  frc::SmartDashboard::PutNumber("Nav-X Yaw", m_gyro.GetYaw());

#ifdef RAMSETE_LOG
  std::cout << "RLog," << frc::RobotController::GetFPGATime() << "," << m_ramseteLog.m_poseX << "," << m_ramseteLog.m_poseY << "," << m_ramseteLog.m_poseAngle 
            << "," << m_ramseteLog.m_wheelSpeedL << "," << m_ramseteLog.m_wheelSpeedR << "," << m_ramseteLog.m_wheelVoltageL << "," << m_ramseteLog.m_wheelVoltageR 
            << std::endl;
#endif

m_lastAngle = currentHeading;

}

void DrivetrainSub::setDrivetrainEncoderZero() {
  m_rightMotor1.GetEncoder().SetPosition(0);
  m_rightMotor2.GetEncoder().SetPosition(0);
  m_rightMotor3.GetEncoder().SetPosition(0);
  m_rightMotor4.GetEncoder().SetPosition(0);

  m_leftMotor1.GetEncoder().SetPosition(0);
  m_leftMotor2.GetEncoder().SetPosition(0);
  m_leftMotor3.GetEncoder().SetPosition(0);
  m_leftMotor4.GetEncoder().SetPosition(0);
}

void DrivetrainSub::resetOdometry(frc::Pose2d pose) {
  setDrivetrainEncoderZero();
  m_odometry.ResetPosition(pose, pose.Rotation());
}

void DrivetrainSub::arcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
  //printf("fwd=%4.2f rot=%4.2f\n", fwd, rot);
}

void DrivetrainSub::shiftUp() {
   m_shifter.Set(true);
   frc::SmartDashboard::PutBoolean("High Gear", true);
}

void DrivetrainSub::shiftDown() {
  m_shifter.Set(false);
  frc::SmartDashboard::PutBoolean("High Gear", false);
}

bool DrivetrainSub::isShifterInHighGear() {
  return m_shifter.Get();
}

void DrivetrainSub::tankDriveVolts(units::volt_t leftVolts, units::volt_t rightVolts) {
#ifdef RAMSETE_LOG
  m_ramseteLog.m_wheelVoltageL = leftVolts;
  m_ramseteLog.m_wheelVoltageR = rightVolts;
#endif
  m_leftMotors.SetVoltage(-leftVolts);
  m_rightMotors.SetVoltage(rightVolts);
  m_drive.Feed();
}

void DrivetrainSub::tankDrive(double left, double right) {
  m_leftMotors.Set(-left);
  m_rightMotors.Set(right);
}

double DrivetrainSub::getAverageEncoderDistance() {
  return (getLeftEncoderDistanceM() + getRightEncoderDistanceM()) / 2.0;
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

frc::Pose2d DrivetrainSub::getPose() {
  frc::Pose2d currentPose = m_odometry.GetPose();
#ifdef RAMSETE_LOG
  m_ramseteLog.m_poseX = currentPose.Translation().X();
  m_ramseteLog.m_poseY = currentPose.Translation().Y();
  m_ramseteLog.m_poseAngle = currentPose.Rotation().Degrees();
#endif
  return currentPose; //At 0 degrees, forward is positive x, left is positive y (location is relative to the center of the robot)
}

frc::DifferentialDriveWheelSpeeds DrivetrainSub::getWheelSpeeds() {
#ifdef RAMSETE_LOG
  m_ramseteLog.m_wheelSpeedL = units::meters_per_second_t(getLeftVelocity());
  m_ramseteLog.m_wheelSpeedR = units::meters_per_second_t(getRightVelocity());
#endif
  return {units::meters_per_second_t(getLeftVelocity()),
          units::meters_per_second_t(getRightVelocity())};
}

// Returns the conversion factor for converting motor encoder ticks to meters.
// This method takes into account which gear we are in.s
double DrivetrainSub::getEncoderRotationsToM() {
  double conversionFactor;

  if(isShifterInHighGear()) {
    conversionFactor = kEncoderRotationsToMHighGear;
  }
  else {
    conversionFactor = kEncoderRotationsToMLowGear;
  }
  return conversionFactor;
}

// Get the distance from the left and right encoders.  Distance is in meters.
// These methods take into account which gear we are in.
double DrivetrainSub::getLeftEncoderDistanceM()
{
    return (getLeftEncoderRaw() * getEncoderRotationsToM());
}

double DrivetrainSub::getRightEncoderDistanceM()
{
  return (getRightEncoderRaw() * getEncoderRotationsToM());
}

// Get the raw left and right encoder values.  Returns rotations.
double DrivetrainSub::getLeftEncoderRaw()
{
  return (-m_leftMotor1.GetEncoder().GetPosition());
}

double DrivetrainSub::getRightEncoderRaw()
{
  return (m_rightMotor1.GetEncoder().GetPosition());
}

double DrivetrainSub::getLeftVelocity()
{
  auto encoder = m_leftMotor1.GetEncoder();
  
  return (-encoder.GetVelocity() * getEncoderRotationsToM() / 60.0 );
}
double DrivetrainSub::getRightVelocity()
{ 
  auto encoder = m_rightMotor1.GetEncoder();
  return (encoder.GetVelocity() * getEncoderRotationsToM() / 60.0 );
}

void DrivetrainSub::autoShift() {
  if(m_autoShiftEnabled) {
    double avgWheelSpeeds = (getLeftVelocity() + getRightVelocity()) / 2;
    if(avgWheelSpeeds < 0) {
      avgWheelSpeeds *= -1.0;
    }
    if(avgWheelSpeeds > kShiftUpSpeed) {
      shiftUp();
    }
    else if(avgWheelSpeeds < kShiftDownSpeed) {
      shiftDown();
    }
  }
}

void DrivetrainSub::disableAutoShift() {
  m_autoShiftEnabled=false;
}

void DrivetrainSub::enableAutoShift() {
  m_autoShiftEnabled=true;
}