/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ClimberSub.h"
#include "Constants.h"

ClimberSub::ClimberSub() :
  m_armMotor{CanIds::kElevatorMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
  m_climbReleaseLatch{PneumaticIds::kClimbReleaseLatch},
  m_climbBalanceMotor{CanIds::kClimbBalanceMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless} {
  init();
}

void ClimberSub::init() {
  releaseLatch(false);
  m_armMotor.GetEncoder().SetPosition(0);
}

void ClimberSub::Periodic() {}

void ClimberSub::releaseLatch(bool position) {
  m_climbReleaseLatch.Set(!position);
}

void ClimberSub::setWinchPower(double power) {
  m_armMotor.Set(-power);
}

void ClimberSub::moveOnGenSwitch(double power) {
  m_climbBalanceMotor.Set(power);
}

double ClimberSub::getArmMotorEncoder() {
  return (-1 * m_armMotor.GetEncoder().GetPosition());
}

int ClimberSub::getOperatorShiftState(frc::Joystick* joystick) { 
  // POV position is reported in degrees with 0 deg being up and increasing clockwise
  // Positions between Up/down/left/right are ignored (i.e. 45 deg)
  return joystick->GetPOV(0);
}
