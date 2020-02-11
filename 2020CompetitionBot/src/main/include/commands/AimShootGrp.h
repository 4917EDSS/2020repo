/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/VisionSub.h"
#include "subsystems/DrivetrainSub.h"
#include "subsystems/IntakeSub.h"
#include "subsystems/ShooterSub.h"

class AimShootGrp
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AimShootGrp> {
 public:
  AimShootGrp(VisionSub* visionSub, DrivetrainSub* drivetrainSub, bool isFar, ShooterSub* shooterSub, IntakeSub* intakeSub);
};
