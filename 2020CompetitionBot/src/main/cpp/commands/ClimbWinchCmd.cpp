/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include "commands/ClimbWinchCmd.h"
#include "Constants.h"

ClimbWinchCmd::ClimbWinchCmd(ClimberSub* climbSub, frc::Joystick* joystick)
  : m_climbSub(climbSub),
    m_joystick(joystick)
  {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climbSub});
}

// Called when the command is initially scheduled.
// We just turn the power on and leave it until command is interrupted calling End
void ClimbWinchCmd::Initialize() {
  // Get inisual incoder for start configuration
  // This will be the minimum incoder value allowed when retracting the arm
  m_minimumArmMotorEncoderValue = 0;
}

// Called repeatedly when this Command is scheduled to run
void ClimbWinchCmd::Execute() {
  double p = -1 * (m_joystick->GetThrottle());
  if(fabs(p) < 0.1) {
    p = 0;
  }
  double e = -1 * (m_climbSub->getArmMotorEncoderRaw());
  frc::SmartDashboard::PutNumber("Climb Winch Encoder", e);

  bool isShiftDownPressed = (m_climbSub->getOperatorShiftState(m_joystick) == DpadConstants::kDown);

  // Apply power
  if(isShiftDownPressed) {
    m_climbSub->setWinchPower(p);
  }
  else if((p > 0) && (e >= ClimbConstants::kMaxArmMotorEncoderValue)) {
    m_climbSub->setWinchPower(0.0);
  }
  else if ((p < 0) && (e <= 0)) {
    m_climbSub->setWinchPower(0.0);
  }
  else if (p < 0 && e < 10) {
    m_climbSub->setWinchPower(p/3);
  }
  else {
    m_climbSub->setWinchPower(p);
  }
}

// Relying on command inrerruption to end this
void ClimbWinchCmd::End(bool interrupted) {
}

// Returns true when the command should end.
bool ClimbWinchCmd::IsFinished() { return false; }
