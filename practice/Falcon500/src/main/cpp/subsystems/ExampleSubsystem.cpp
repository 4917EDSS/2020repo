/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/ExampleSubsystem.h"


ExampleSubsystem::ExampleSubsystem()
  : m_falcon{25} {
  // Implementation of subsystem constructor goes here.
  frc::SmartDashboard::PutNumber("motorSpeed", 0.0);
}

void ExampleSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  double speedToSet = frc::SmartDashboard::GetNumber("motorSpeed", getSpeed());
  setSpeed(speedToSet);
  //frc::SmartDashboard::PutNumber("motorSpeed", getSpeed());

  frc::SmartDashboard::PutNumber("motorCurrent", m_falcon.GetOutputCurrent());
}

void ExampleSubsystem::setSpeed(double speed) {
  m_falcon.Set(speed);
}

double ExampleSubsystem::getSpeed() {
  return m_falcon.Get();
}