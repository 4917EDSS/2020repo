/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/trajectory/Trajectory.h>
#include "subsystems/DrivetrainSub.h"
#include <frc2/command/RamseteCommand.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RamseteCmd
    : public frc2::RamseteCommand{
 public:
  RamseteCmd(Trajectory t, DrivetrainSub* drivetrainsub, bool isReset);
  
  void Initialize() override;
  void End(bool interrupted) override;
  
 private:
  DrivetrainSub* m_drivetrainSub;
  bool m_isReset;
};
