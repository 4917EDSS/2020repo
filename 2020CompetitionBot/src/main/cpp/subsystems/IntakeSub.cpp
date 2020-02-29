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
      m_frontIntakeSensorL{DioIds::kFrontIntakeSensorL}, 
      m_frontIntakeSensorR{DioIds::kFrontIntakeSensorR}, 
      m_magazineFrontSensorL{DioIds::kMagazineFrontSensorL},
      m_magazineFrontSensorR{DioIds::kMagazineFrontSensorR},
      m_magazineFullSensor{DioIds::kMagazineFullSensor}     {
}
//positive takes balls in negative takes balls to shooter 

void IntakeSub::init() {
}

void IntakeSub::Periodic() {}

void IntakeSub::setFrontRollerIntakePower(double power) {
    m_frontRollerIntakeMotor.Set(ControlMode::PercentOutput, power);

}

void IntakeSub::setMagazineIntakePower(double power) {
    m_topIntakeMotor.Set(ControlMode::PercentOutput, power);
}

bool IntakeSub::getFrontIntakeSensor() {
    if(!m_frontIntakeSensorL.Get() || !m_frontIntakeSensorR.Get()){
        return true;
    }
    else{
        return false;
    }
}

bool IntakeSub::getMagazineFrontSensor() {
    if(!m_magazineFrontSensorL.Get() || !m_magazineFrontSensorR.Get()){
        return true;
    }
    else{
        return false;
    }
}

bool IntakeSub::getMagazineFullSensor() {
    return !m_magazineFullSensor.Get();
}
