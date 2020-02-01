/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ClimberSub.h"
#include "Constants.h"

ClimberSub::ClimberSub() :
    m_elevatorMotor1{CanIds::kElevatorMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_climbReleaseLatch{PneumaticIds::kClimbReleaseLatch},
    m_climbBalanceMotor{CanIds::kClimbBalanceMotor} {
}

// This method will be called once per scheduler run
void ClimberSub::Periodic() {}

void ClimberSub::togglePosition(bool position) {
    m_climbReleaseLatch.Set(position);
}

void ClimberSub::setWinchSpeed(double speed) {
    m_elevatorMotor1.Set(speed);
}

void ClimberSub::moveOnGenSwitch(double power) {
    m_climbBalanceMotor.Set(power);
}