/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ClimberSub.h"

ClimberSub::ClimberSub() {

elevatorMotor1.reset(new rev::CANSparkMax(13, rev::CANSparkMaxLowLevel::MotorType::kBrushless));

 climbReleaseSolenoid.reset(new frc::Solenoid(14));
}
// This method will be called once per scheduler run
void ClimberSub::Periodic() {}
