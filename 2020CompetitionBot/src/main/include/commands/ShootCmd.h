/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterSub.h"
#include "subsystems/IntakeSub.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */

class ShootCmd
    : public frc2::CommandHelper<frc2::CommandBase, ShootCmd> {
 public:

  ShootCmd(ShooterSub* shooterSub, IntakeSub* intakeSub, bool isFar);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;

  private:
  ShooterSub* m_shooterSub;
  IntakeSub* m_intakeSub;
  double m_targetSpeed;
  double m_lastDiff;
  uint64_t m_lastTime;
  bool m_isFar;
  double powers[5];
  int m_index=0;
  double m_integralDiff;
  double runPID();


};
