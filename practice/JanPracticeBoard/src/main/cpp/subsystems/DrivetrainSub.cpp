/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DrivetrainSub.h"

constexpr int LEFT_DRIVE_MOTOR_1_CAN_ID = 2;
constexpr int RIGHT_DRIVE_MOTOR_1_CAN_ID = 22;

DrivetrainSub::DrivetrainSub() 
    : m_leftMotor1{LEFT_DRIVE_MOTOR_1_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
      m_rightMotor1{RIGHT_DRIVE_MOTOR_1_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless} {

}

// This method will be called once per scheduler run
void DrivetrainSub::Periodic() {
    drive(0.1, 0.1);        // TODO:  Remove this!
}

void DrivetrainSub::drive(double lPower, double rPower) {
    m_leftMotor1.Set(lPower);
    m_rightMotor1.Set(rPower);
}
