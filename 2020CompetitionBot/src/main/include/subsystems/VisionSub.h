/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>

namespace VisionConstants {
    constexpr double kXAllignmentTolerence = 0.5;
    constexpr double kXMax = 30.0;
    constexpr double kFrontCameraId = 1;
}

class VisionSub : public frc2::SubsystemBase {
 public:
  VisionSub();
  void init();
  void Periodic();
  double getVisionTarget();
  double getVerticalOffset();
  void setFarVisionPipeline();
  void setShortVisionPipeline();
  void setNeutralVisionPipeline();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
};
