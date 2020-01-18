/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/IntakeSub.h"

IntakeSub::IntakeSub() {
    
    FrontIntakeMotor.reset(new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(1));
    InteriorIntakeMotor.reset(new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(4));

}

void IntakeSub::SetIntake(double speed){

    FrontIntakeMotor->Set(ControlMode::PercentOutput, speed);
    InteriorIntakeMotor->Set(ControlMode::PercentOutput, speed);

}

// This method will be called once per scheduler run
void IntakeSub::Periodic() {}


//Ball enters robot through the gap in bumper
//Motors on rollers suck the ball into robot
//Ball enters shooter



