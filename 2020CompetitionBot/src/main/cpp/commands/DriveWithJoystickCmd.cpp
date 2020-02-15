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
constexpr double kMaxForwardAccel = 0.02;
constexpr double kMaxTurnAccel = 0.1;
DriveWithJoystickCmd::DriveWithJoystickCmd(DrivetrainSub* drivetrainSub, frc::Joystick* joystick) : m_drivetrainSub(drivetrainSub), m_joystick(joystick){
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrainSub});
}

// Called when the command is initially scheduled.
void DriveWithJoystickCmd::Initialize() {
  m_forwardPowerPrevious = 0.0;
  m_turnPowerPrevious = 0.0;
}

double adjustSensitivity (double power) {
  double sign=-1.0;
  if (power >= 0) {
   sign=1.0;
  }
  power=pow(power,kSensativityPower);
  power=fabs(power)*sign;

  return power;
}

double capAcceleration (double power, double powerPrevious, double maxAccel){
  if (powerPrevious - power > maxAccel){
    power = powerPrevious - maxAccel;
  }
  else if (power - powerPrevious > maxAccel){
    power = powerPrevious + maxAccel;
  }
  return power;
}
// Called repeatedly when this Command is scheduled to run
void DriveWithJoystickCmd::Execute() {
  double forwardPower=m_joystick->GetY();
  double turnPower=-m_joystick->GetZ();

  forwardPower =  adjustSensitivity (forwardPower);
  turnPower = adjustSensitivity (turnPower);

  forwardPower = capAcceleration(forwardPower, m_forwardPowerPrevious, kMaxForwardAccel);
  turnPower = capAcceleration(turnPower, m_turnPowerPrevious, kMaxTurnAccel);

  m_drivetrainSub->arcadeDrive(forwardPower,turnPower);
  m_drivetrainSub->autoShift();

  m_forwardPowerPrevious = forwardPower;
  m_turnPowerPrevious = turnPower;
 }

// Called once the command ends or is interrupted.
void DriveWithJoystickCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveWithJoystickCmd::IsFinished() { return false; }
