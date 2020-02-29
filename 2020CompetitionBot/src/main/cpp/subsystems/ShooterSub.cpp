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
    m_hoodMotor(CanIds::kHoodMotor){
      m_shooterMotor1.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);
      m_shooterMotor2.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);
      m_hoodMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder,0,0);
      m_hoodMotor.SetSelectedSensorPosition(0);
      m_shooterMotor1.ConfigVelocityMeasurementPeriod(ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_5Ms);
      m_shooterMotor1.ConfigVelocityMeasurementWindow(4);
      m_shooterMotor2.ConfigVelocityMeasurementPeriod(ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_5Ms);
      m_shooterMotor2.ConfigVelocityMeasurementWindow(4);
  // Implementation of subsystem constructor goes here.
}

void ShooterSub::init() {
}

void ShooterSub::Periodic() {
  // Implementation of subsystem periodic method goes here.
  frc::SmartDashboard::PutNumber("ShooterSpeed", getSpeed());
  frc::SmartDashboard::PutNumber("Hood Encoder Value",getHoodEncoder());
}

//Sets speed of all motors
void ShooterSub::setSpeed(double speed) {
  m_shooterMotor1.Set(-speed);
  m_shooterMotor2.Set(speed);
}


void ShooterSub::setHoodSpeed(double hoodSpeed) {
  m_hoodMotor.Set(-hoodSpeed);
}

double ShooterSub::getHoodEncoder() {
  return m_hoodMotor.GetSelectedSensorPosition();
}

// Gets maximum absolute speeds of both motors
int ShooterSub::getSpeed() {
  int motor2Speed =  m_shooterMotor2.GetSelectedSensorVelocity();
  return std::abs(motor2Speed);
}