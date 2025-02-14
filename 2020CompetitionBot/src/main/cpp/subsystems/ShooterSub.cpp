/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/ShooterSub.h"
#include <cmath>
#include "Constants.h"




ShooterSub::ShooterSub()
  : m_shooterMotor1(CanIds::kShootMotor1), 
    m_shooterMotor2(CanIds::kShootMotor2), 
    m_feederMotor(CanIds::kFeederMotor), 
    m_hoodAdjuster(PneumaticIds::kHoodAdjuster) {
  m_shooterMotor1.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);
  m_shooterMotor2.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);
  m_shooterMotor1.ConfigVelocityMeasurementPeriod(ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_5Ms);
  m_shooterMotor1.ConfigVelocityMeasurementWindow(4);
  m_shooterMotor2.ConfigVelocityMeasurementPeriod(ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_5Ms);
  m_shooterMotor2.ConfigVelocityMeasurementWindow(4);
}

void ShooterSub::init() {
  setPower(0.0);
  //frc::SmartDashboard::PutNumber("FlySpeed", 15800);
  flipHoodUp(false);

}

void ShooterSub::Periodic() {
  // Implementation of subsystem periodic method goes here.
  frc::SmartDashboard::PutNumber("Shooter Speed Ticks", getSpeed());

}

// Sets speed of all motors
void ShooterSub::setPower(double power) {
  m_shooterMotor1.SetVoltage(-power * 12_V);
  m_shooterMotor2.SetVoltage(power * 12_V);
}

bool ShooterSub::getHoodPosition() {
  return !m_hoodAdjuster.Get();
}

void ShooterSub::flipHoodUp(bool location){
  m_hoodAdjuster.Set(!location);
  frc::SmartDashboard::PutBoolean("Hood Up", location);
  std::cout << "Hood Up: " << location << std::endl;
}

// Gets maximum absolute speeds of both motors
int ShooterSub::getSpeed() {
  int motor2Speed =  m_shooterMotor2.GetSelectedSensorVelocity();
  return std::abs(motor2Speed);
}