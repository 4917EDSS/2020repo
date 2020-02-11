/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cmath>
#include <frc/Joystick.h>
#include "commands/DriveWithJoystickCmd.h"
#include "subsystems/DrivetrainSub.h"
constexpr int kSensativityPower=2;

DriveWithJoystickCmd::DriveWithJoystickCmd(DrivetrainSub* drivetrainSub, frc::Joystick* joystick) : m_drivetrainSub(drivetrainSub), m_joystick(joystick){
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrainSub});
}

// Called when the command is initially scheduled.
void DriveWithJoystickCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystickCmd::Execute() {
  double y=m_joystick->GetY();
  double SignY=-1.0;
  if (y >= 0) {
   SignY=1.0;
  }
  y=pow(y,kSensativityPower);
  y=fabs(y)*SignY;
  double z=-m_joystick->GetZ();
  double SignZ=-1.0;
  if (z >= 0) {
   SignZ=1.0;
  }
  z=pow(z,kSensativityPower);
  z=fabs(z)*SignZ;
  m_drivetrainSub->arcadeDrive(y,z);
  m_drivetrainSub->autoShift();
 }

// Called once the command ends or is interrupted.
void DriveWithJoystickCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveWithJoystickCmd::IsFinished() { return false; }
