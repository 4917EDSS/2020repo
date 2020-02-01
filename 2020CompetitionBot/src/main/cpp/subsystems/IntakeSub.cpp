/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/IntakeSub.h"
#include "Constants.h"

IntakeSub::IntakeSub() 
    : m_frontIntakeMotor{CanIds::kFrontIntakeMotor},
      m_interiorIntakeMotor{CanIds::kInteriorIntakeMotor},
      m_magazineFullSensor{DioIds::kMagazineFullSensor} {
    
}
//positive takes balls in negative takes balls to shooter 
void IntakeSub::setIntake(double speed) {
    if(speed < 0) {
        m_frontIntakeMotor.Set(ControlMode::PercentOutput,-speed);
    }
    else {
        m_frontIntakeMotor.Set(ControlMode::PercentOutput, speed);
    }

    m_interiorIntakeMotor.Set(ControlMode::PercentOutput, speed);
}

// This method will be called once per scheduler run
void IntakeSub::Periodic() {}


//Ball enters robot through the gap in bumper
//Motors on rollers suck the ball into robot
//Ball enters shooter
