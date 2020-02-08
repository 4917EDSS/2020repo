/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AimShootGrp.h"
#include "commands/AimSpinFlywheelGrp.h"
#include "commands/ShootCmd.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
AimShootGrp::AimShootGrp(VisionSub* visionSub, DrivetrainSub* drivetrainSub, bool isFar) { 
  // Add your commands here, e.g.
AddCommands(AimSpinFlywheelGrp(visionSub, drivetrainSub, isFar)/*, ShootCmd()*/); //need to add shoot Cmd with it's parameters
  // AddCommands(FooCommand(), BarCommand());
}
