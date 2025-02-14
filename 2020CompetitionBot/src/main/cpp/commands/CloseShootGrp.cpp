/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/CloseShootGrp.h"
#include "commands/ShootCmd.h"
#include "commands/SpinFlywheelCmd.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
CloseShootGrp::CloseShootGrp(IntakeSub* intakeSub, ShooterSub* shooterSub) {
  // Add your commands here, e.g.
  
  AddCommands(SpinFlywheelCmd(shooterSub, false), ShootCmd(shooterSub, intakeSub, false, false));
}
