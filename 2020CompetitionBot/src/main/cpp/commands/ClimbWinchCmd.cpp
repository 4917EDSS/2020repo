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

// This is the proportional controller approach. We are trying a different approach
// constexpr double kTolerance = 2.0;
// constexpr double kControlStartDelta = 10.0;
// constexpr double kMinPower = 0.05;
// constexpr double kMaxPower = 1.0;
constexpr double kMinArmMotorEncoderValue = 10.0;
constexpr double kMaxArmMotorEncoderValue = 471.0;


ClimbWinchCmd::ClimbWinchCmd(ClimberSub* climberSub, frc::Joystick* joystick)
  : m_climberSub(climberSub),
    m_joystick(joystick) {

    // This is the proportional controller approach. We are trying a different approach
    // m_pcUp(kMaxArmMotorEncoderValue, kControlStartDelta, kTolerance, kMinPower, kMaxPower), 
    // m_pcDown(kMinArmMotorEncoderValue, kControlStartDelta, kTolerance, kMinPower, kMaxPower) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climberSub});
}

// Called when the command is initially scheduled.
// We just turn the power on and leave it until command is interrupted calling End
void ClimbWinchCmd::Initialize() {
  // Get inisual incoder for start configuration
  // This will be the minimum incoder value allowed when retracting the arm
}

// Called repeatedly when this Command is scheduled to run
void ClimbWinchCmd::Execute() {
  double pRaw = -1 * (m_joystick->GetThrottle());
  // Accomodate drift on controller joystick by ignoring tiny values.
  if (fabs(pRaw) < 0.1) {
    pRaw = 0;
  }
  double e = (m_climberSub->getArmMotorEncoder());
  frc::SmartDashboard::PutNumber("Climb Winch Encoder", e);

  bool isShiftDownPressed = (m_climberSub->getOperatorShiftState(m_joystick) == DpadConstants::kDown);
  
  // ----------------------------------------------------------------------------------------
  // This is the proportional controller approach. We are trying a different approach
  // ----------------------------------------------------------------------------------------
  // Apply power
  // if (isShiftDownPressed) {
  //   m_climbSub->setWinchPower(pRaw);
  // } else {
  //   double pProportional;
  //   if (pRaw < 0) {
  //     pProportional = m_pcDown.getPower(e);
  //   } else {
  //     pProportional = m_pcUp.getPower(e);
  //   }
  //   m_climbSub->setWinchPower(pProportional);  
  // }
  // ----------------------------------------------------------------------------------------

  // Apply power
  if (isShiftDownPressed) {
    m_climberSub->setWinchPower(pRaw);
  }
  else if ((pRaw > 0) && (e >= kMaxArmMotorEncoderValue)) {
    m_climberSub->setWinchPower(0.0);
  }
  else if (pRaw > 0 && e >= kMaxArmMotorEncoderValue - 10) {
    m_climberSub->setWinchPower(pRaw / (11 - (kMaxArmMotorEncoderValue - e)));
  }
  else if ((pRaw < 0) && (e <= 0)) {
    m_climberSub->setWinchPower(0.0);
  }
  else if (pRaw < 0 && e <= 10) {
    m_climberSub->setWinchPower(pRaw / (11 - (kMaxArmMotorEncoderValue - e)));
  }
  else {
    m_climberSub->setWinchPower(pRaw);
  }
}

// Relying on command inrerruption to end this
void ClimbWinchCmd::End(bool interrupted) {
}

// Returns true when the command should end.
bool ClimbWinchCmd::IsFinished() { return false; }
