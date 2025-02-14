/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc2/command/InstantCommand.h>

#include "commands/AimSpinupShootGrp.h"
#include "commands/VisionAlignmentCmd.h"
#include "commands/SpinFlywheelCmd.h"
#include "commands/ShootCmd.h"


// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
AimSpinupShootGrp::AimSpinupShootGrp(VisionSub* visionSub, DrivetrainSub* drivetrainSub, ShooterSub* shooterSub, IntakeSub* intakeSub,bool isFar)
  : CommandHelper(
          // The deadline command
          VisionAlignmentCmd(visionSub, drivetrainSub, isFar), 
          SpinFlywheelCmd(shooterSub, isFar)) {
  AddCommands(SpinFlywheelCmd(shooterSub, false), ShootCmd(shooterSub, intakeSub, false, false));
}
