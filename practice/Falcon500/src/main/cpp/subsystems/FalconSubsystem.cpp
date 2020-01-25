/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/FalconSubsystem.h"


FalconSubsystem::FalconSubsystem()
  : m_falcon1(25), 
    m_falcon2(26) {
  // Implementation of subsystem constructor goes here.
}
  
void FalconSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  double speedToSet = frc::SmartDashboard::GetNumber("motorSpeed1", getSpeed());
  setSpeed(speedToSet);
  //frc::SmartDashboard::PutNumber("motorSpeed", getSpeed());
  frc::SmartDashboard::PutNumber("motorCurrent1", m_falcon1.GetOutputCurrent());
  frc::SmartDashboard::PutNumber("motorCurrent2", m_falcon2.GetOutputCurrent());
}

void FalconSubsystem::setSpeed(double speed) {
  m_falcon1.Set(speed);
  m_falcon2.Set(speed);
}

double FalconSubsystem::getSpeed() {
  return m_falcon1.Get();
}
