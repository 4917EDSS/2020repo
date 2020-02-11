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

constexpr float kEncoderRotationsToMLowGear = 5.0/(160.162);
constexpr float kEncoderRotationsToMHighGear = 5.0/(102.264);
constexpr double kShiftUpSpeed = 2.5;
constexpr double kShiftDownSpeed = 1.0;

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

// TODO: Do we need encoder to be on the output shaft vs on the motor?
// Set the distance per pulse for the encoders
// WARNING!  This value can get erased during brownouts.  Safer to do the conversion in the getLeftEncoder and getRightEncoder functions TODO.
   m_shifter.Set(false);
  setDrivetrainEncoderZero();

  m_rightMotor1.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);
  m_rightMotor2.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);
  m_rightMotor3.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);
  m_rightMotor4.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);

  m_leftMotor1.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);
  m_leftMotor2.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);
  m_leftMotor3.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);
  m_leftMotor4.SetSmartCurrentLimit(DriveConstants::kSmartCurrentLimit);


  frc::SmartDashboard::PutNumber("drive power", 0);

  arcadeDrive(0.2, 0);
}

// This method will be called once per scheduler run
void DrivetrainSub::Periodic() {

  m_odometry.Update(frc::Rotation2d(units::degree_t(getHeading())),
                    units::meter_t(getLeftEncoderDistanceM()),
                    units::meter_t(getRightEncoderDistanceM()));


  m_drive.Feed();

  frc::SmartDashboard::PutNumber("RawEnc R", getRightEncoderRaw());
  frc::SmartDashboard::PutNumber("RawEnc L", getLeftEncoderRaw());
  frc::SmartDashboard::PutNumber("CnvrtdEnc R", getRightEncoderDistanceM());
  frc::SmartDashboard::PutNumber("CnvrtdEnc L", getLeftEncoderDistanceM());
  frc::SmartDashboard::PutNumber("MtrVlcty R", getRightVelocity());
  frc::SmartDashboard::PutNumber("MtrVlcty L", getLeftVelocity());
  frc::SmartDashboard::PutBoolean("High Gear", isShifterInHighGear());
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
  //printf("fwd=%4.2f rot=%4.2f\n", fwd, rot);
}

void DrivetrainSub::shiftUp() {
   m_shifter.Set(true);
}

void DrivetrainSub::shiftDown() {
  m_shifter.Set(false);
}

bool DrivetrainSub::isShifterInHighGear() {
  return m_shifter.Get();
}

void DrivetrainSub::tankDriveVolts(double left, double right) {
  m_leftMotors.Set(left);
  m_rightMotors.Set(-right);
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

frc::Pose2d DrivetrainSub::getPose() { return m_odometry.GetPose(); }

frc::DifferentialDriveWheelSpeeds DrivetrainSub::getWheelSpeeds() {
  return {units::meters_per_second_t(getLeftEncoderDistanceM()),
          units::meters_per_second_t(getRightEncoderDistanceM())};
}

void DrivetrainSub::resetOdometry(frc::Pose2d pose) {
  setDrivetrainEncoderZero();
  m_odometry.ResetPosition(pose,
                           frc::Rotation2d(units::degree_t(getHeading())));
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