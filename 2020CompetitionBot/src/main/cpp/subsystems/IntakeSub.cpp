/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/IntakeSub.h"
#include "Constants.h"

IntakeSub::IntakeSub() 
    : m_frontRollerIntakeMotor{ctre::phoenix::motorcontrol::can::WPI_VictorSPX(CanIds::kFrontRollerIntakeMotor)},
      m_topIntakeMotor{ctre::phoenix::motorcontrol::can::WPI_VictorSPX(CanIds::kTopIntakeMotor)},
      m_bottomIntakeMotor{ctre::phoenix::motorcontrol::can::WPI_VictorSPX(CanIds::kBottomIntakeMotor)},
      m_magazineFullSensor{frc::DigitalInput(DioIds::kMagazineFullSensor)},
      m_frontIntakeSensor{frc::DigitalInput(DioIds::kFrontIntakeSensor)} {
    
}
//positive takes balls in negative takes balls to shooter 

// This method will be called once per scheduler run
void IntakeSub::Periodic() {}

void IntakeSub::setFrontRollerIntakePower(double power) {
    m_frontRollerIntakeMotor.Set(ControlMode::PercentOutput, power);

}

void IntakeSub::setMagazineIntakePower(double power) {
    m_topIntakeMotor.Set(ControlMode::PercentOutput, -power);
    m_bottomIntakeMotor.Set(ControlMode::PercentOutput, power);
}

bool IntakeSub::getFrontIntakeSensor(){
    return m_frontIntakeSensor.Get();
}

bool IntakeSub::getMagazineFullSensor(){
    return m_magazineFullSensor.Get();
}
//Ball enters robot through the gap in bumper
//Motors on rollers suck the ball into robot
//Ball enters shooter
