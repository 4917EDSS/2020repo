/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/VisionSub.h"
#include <networktables/NetworkTableInstance.h>

VisionSub::VisionSub() {}

// This method will be called once per scheduler run
void VisionSub::Periodic() {}

double VisionSub::getVisionTarget(int camera) {
  return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
}

double VisionSub::getVerticalOffset(int camera){
	return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
}

