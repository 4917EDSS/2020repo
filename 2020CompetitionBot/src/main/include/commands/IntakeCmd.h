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
#include "subsystems/drivetrainSub.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class IntakeCmd
    : public frc2::CommandHelper<frc2::CommandBase, IntakeCmd> {
 public:
  IntakeCmd(IntakeSub* intakeSub, DrivetrainSub* drivetrainSub);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;


private:
  IntakeSub* m_intakeSub;
  DrivetrainSub* m_drivetrainSub;
  int m_state;
  double m_startingEncDistance;

  /*
  state 0: No ball inside the intake
  state 1: 1 ball in intake, banana not full
  state 2: 1 ball in intake, banana full
  */
};
