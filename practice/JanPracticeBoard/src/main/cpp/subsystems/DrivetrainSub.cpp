/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Constants.h"
#include "subsystems/DrivetrainSub.h"

DrivetrainSub::DrivetrainSub() 
    : m_leftMotor1{canids::LEFT_DRIVE_MOTOR_1, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_rightMotor1{canids::RIGHT_DRIVE_MOTOR_1, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_auxMotor{canids::AUX_MOTOR} {

}

// This method will be called once per scheduler run
void DrivetrainSub::Periodic() {}

void DrivetrainSub::drive(double lPower, double rPower) {
    m_leftMotor1.Set(lPower);
    m_rightMotor1.Set(rPower);
}

// Added motor to drivetrain to be able to use the VictorSPX
void DrivetrainSub::setAuxPower(double power) {
    m_auxMotor.Set(power);
}
