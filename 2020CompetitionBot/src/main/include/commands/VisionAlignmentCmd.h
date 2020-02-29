/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/VisionSub.h"
#include "subsystems/DrivetrainSub.h"
#include "components/ProportionalController.h"

class VisionAlignmentCmd
    : public frc2::CommandHelper<frc2::CommandBase, VisionAlignmentCmd> {
 public:
  VisionAlignmentCmd(VisionSub* visionSub, DrivetrainSub* drivetrainSub, bool m_isFar);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  VisionSub* m_visionSub;
  DrivetrainSub* m_drivetrainSub;
  uint64_t m_startTime;
  bool m_isFar;
  double m_lastX;
  ProportionalController m_pController;
};
