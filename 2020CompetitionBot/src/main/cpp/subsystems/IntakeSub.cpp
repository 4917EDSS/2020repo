/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/IntakeSub.h"
#include "Constants.h"

IntakeSub::IntakeSub() 
    : m_frontRollerIntakeMotor{CanIds::kFrontRollerIntakeMotor},
      m_topIntakeMotor{CanIds::kTopIntakeMotor},
      m_frontIntakeSensor{DioIds::kFrontIntakeSensor}, 
      m_PowerCellSensor1{DioIds::kPowerCellSensor1},
      m_PowerCellSensor2{DioIds::kPowerCellSensor2},
      m_PowerCellSensor3{DioIds::kPowerCellSensor3},
      m_PowerCellSensor4{DioIds::kPowerCellSensor4}     {
    
}
//positive takes balls in negative takes balls to shooter 

// This method will be called once per scheduler run
void IntakeSub::Periodic() {
    frc::SmartDashboard::PutBoolean("Intake Sensor", getFrontIntakeSensor());
}

void IntakeSub::setFrontRollerIntakePower(double power) {
    m_frontRollerIntakeMotor.Set(ControlMode::PercentOutput, power);

}

void IntakeSub::setMagazineIntakePower(double power) {
    m_topIntakeMotor.Set(ControlMode::PercentOutput, power);
}

bool IntakeSub::getFrontIntakeSensor() {
    return !m_frontIntakeSensor.Get();
}

// These need to be added to IntakeCmd
bool IntakeSub::getPowerCellSensor1() {
    return !m_PowerCellSensor1.Get();
}

bool IntakeSub::getPowerCellSensor2() {
    return !m_PowerCellSensor2.Get();
}

bool IntakeSub::getPowerCellSensor3() {
    return !m_PowerCellSensor3.Get();
}

bool IntakeSub::getPowerCellSensor4() {
    return !m_PowerCellSensor4.Get();
}