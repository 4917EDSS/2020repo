/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ControlPanelSub.h"
#include "Constants.h"

ControlPanelSub::ControlPanelSub() : m_controlPanelMotor(CanIds::kControlPanelMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
                                     m_controlPanelFlipper(PneumaticIds::kControlPanelFlipper){
}

// This method will be called once per scheduler run
void ControlPanelSub::Periodic() {}

void ControlPanelSub::togglePosition(bool position){
    m_controlPanelFlipper.Set(position);
}

void ControlPanelSub::setWheelSpeed(double speed){
    m_controlPanelMotor.Set(speed);
}

