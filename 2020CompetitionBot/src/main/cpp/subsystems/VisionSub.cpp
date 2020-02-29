/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/VisionAlignmentCmd.h"
#include "subsystems/VisionSub.h"
#include <networktables/NetworkTableInstance.h>

VisionSub::VisionSub() {
  frc::SmartDashboard::PutNumber("LPower", 0);
}

void VisionSub::init() {
}
// This method will be called once per scheduler run
void VisionSub::Periodic() {}

void VisionSub::setFarVisionPipeline() {
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", 2.0);
}

void VisionSub::setShortVisionPipeline() {
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", 1.0);
}

void VisionSub::setNeutralVisionPipeline() {
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", 0.0);
}

double VisionSub::getVisionTarget() {
  return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
}

double VisionSub::getVerticalOffset(){
	return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
}