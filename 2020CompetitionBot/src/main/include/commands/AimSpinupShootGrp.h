/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include "subsystems/VisionSub.h"
#include "subsystems/DrivetrainSub.h"
#include "subsystems/ShooterSub.h"
#include "subsystems/IntakeSub.h"

class AimSpinupShootGrp
    : public frc2::CommandHelper<frc2::ParallelDeadlineGroup,
                                 AimSpinupShootGrp> {
 public:
  AimSpinupShootGrp(VisionSub* visionSub, DrivetrainSub* drivetrainSub, ShooterSub* shooterSub, IntakeSub* intakeSub, bool isFar);
};
