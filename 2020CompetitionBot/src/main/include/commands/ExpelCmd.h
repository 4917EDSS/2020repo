/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/IntakeSub.h"
#include "subsystems/DrivetrainSub.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ExpelCmd
    : public frc2::CommandHelper<frc2::CommandBase, ExpelCmd> {
 public:
  ExpelCmd(IntakeSub* intakeSub, DrivetrainSub* drivetrainsub);
  
  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
 IntakeSub* m_intakeSub;
 DrivetrainSub* m_drivetrainSub;
};
