/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/ShooterSub.h"
#include <cmath>


ShooterSub::ShooterSub() 
  : m_falcon1(25), 
    m_falcon2(26) {
  // Implementation of subsystem constructor goes here.
}

void ShooterSub::Periodic() {
  // Implementation of subsystem periodic method goes here.
  double speedToSet = frc::SmartDashboard::GetNumber("motorSpeed", getSpeed());
  setSpeed(speedToSet);
  //frc::SmartDashboard::PutNumber("motorSpeed", getSpeed());

  frc::SmartDashboard::PutNumber("motorCurrent", m_falcon1.GetOutputCurrent());
}
//Sets speed of all motors
void ShooterSub::setSpeed(double speed) {
  m_falcon1.Set(speed);
  m_falcon2.Set(speed);
}
// Gets maximum absolute speeds of both motors
double ShooterSub::getSpeed() {
  double falcon1Speed =  m_falcon1.Get();
  std::cout << "ShooterSub::getSpeed - falcon1Speed=";
  std::cout << falcon1Speed;
  std::cout << "\n";

  double falcon2Speed =  m_falcon2.Get();
  std::cout << "ShooterSub::getSpeed - falcon2Speed=";
  std::cout << falcon2Speed;
  std::cout << "\n";

  double overallSpeed = std::max(std::abs(falcon1Speed), std::abs(falcon2Speed));
  std::cout << "ShooterSub::getSpeed - overallSpeed=";
  std::cout << overallSpeed;
  std::cout << "\n";

  return overallSpeed;
}
  
