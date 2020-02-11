/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AimSpinFlywheelGrp.h"
#include "commands/VisionAlignmentCmd.h"

#include <frc2/command/InstantCommand.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
AimSpinFlywheelGrp::AimSpinFlywheelGrp(VisionSub* visionSub, DrivetrainSub* drivetrainSub, bool isFar)
    : CommandHelper(
          // The deadline command
          VisionAlignmentCmd(visionSub, drivetrainSub, isFar)) {
  //AddCommands(Spin flywheel command) replace with real command when it gets made
}
