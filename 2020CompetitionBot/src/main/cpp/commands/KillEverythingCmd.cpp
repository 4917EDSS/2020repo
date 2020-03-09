/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/KillEverythingCmd.h"


KillEverythingCmd::KillEverythingCmd(ClimberSub* climberSub, ControlPanelSub* controlPanelSub,
  DrivetrainSub* driveTrainSub, IntakeSub* intakeSub, ShooterSub* shooterSub, VisionSub* visionSub) 
  : m_shooterSub(shooterSub) {
  
  AddRequirements({climberSub, controlPanelSub, driveTrainSub, intakeSub, shooterSub, visionSub});
}

// Called when the command is initially scheduled.
void KillEverythingCmd::Initialize() {
   m_shooterSub->setPower(0); // TODO:  This shouldn't be necesssary.  Must be missing something in shoot command(s)
}

// Called repeatedly when this Command is scheduled to run
void KillEverythingCmd::Execute() {}

// Called once the command ends or is interrupted.
void KillEverythingCmd::End(bool interrupted) {

}

// Returns true when the command should end.
bool KillEverythingCmd::IsFinished() { return true; }
