/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/IntakeSub.h"
#include "Constants.h"

IntakeSub::IntakeSub() 
    : m_topIntakeMotor{ctre::phoenix::motorcontrol::can::WPI_VictorSPX(CanIds::kTopIntakeMotor)},
      m_bottomIntakeMotor{ctre::phoenix::motorcontrol::can::WPI_VictorSPX(CanIds::kBottomIntakeMotor)},
      m_firstBallSensor{frc::DigitalInput(DioIds::kFirstBallSensor)},
      m_ballIntakeSensor{frc::DigitalInput(DioIds::kBallIntakeSensor)} {
    
}
//positive takes balls in negative takes balls to shooter 

// This method will be called once per scheduler run
void IntakeSub::Periodic() {}

void IntakeSub::setIntake(double speed) {
    m_topIntakeMotor.Set(ControlMode::PercentOutput, -speed);

    m_bottomIntakeMotor.Set(ControlMode::PercentOutput, speed);
}

bool IntakeSub::getBallIntakeSensor(){
    return m_ballIntakeSensor.Get();
}

bool IntakeSub::getFirstBallSensor(){
    return m_firstBallSensor.Get();
}
//Ball enters robot through the gap in bumper
//Motors on rollers suck the ball into robot
//Ball enters shooter
