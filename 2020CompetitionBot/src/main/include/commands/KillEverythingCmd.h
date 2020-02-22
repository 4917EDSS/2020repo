/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimberSub.h"
#include "subsystems/ControlPanelSub.h"
#include "subsystems/DrivetrainSub.h"
#include "subsystems/IntakeSub.h"
#include "subsystems/ShooterSub.h"
#include "subsystems/VisionSub.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class KillEverythingCmd
    : public frc2::CommandHelper<frc2::CommandBase, KillEverythingCmd> {
 public:
  KillEverythingCmd(ClimberSub* climberSub, ControlPanelSub* controlPanelSub, DrivetrainSub* driveTrainSub,
   IntakeSub* IntakeSub, ShooterSub* shooterSub, VisionSub* visionSub);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  private: 


};
