/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ClimberSub.h"

ClimberSub::ClimberSub() : m_elevatorMotor1(13, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
                           m_climbReleaseLatch(14){
}

// This method will be called once per scheduler run
void ClimberSub::Periodic() {}

void ClimberSub::togglePosition(bool position){
    m_climbReleaseLatch.Set(position);
}
