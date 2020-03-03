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

constexpr double kTolerance = 2.0;
constexpr double kControlStartDelta = 10.0;
constexpr double kMinPower = 0.05;
constexpr double kMaxPower = 1.0;
constexpr double kMinArmMotorEncoderValue = 0.0;
constexpr double kMaxArmMotorEncoderValue = 471.0;

ClimbWinchCmd::ClimbWinchCmd(ClimberSub* climbSub, frc::Joystick* joystick)
  : m_climbSub(climbSub),
    m_joystick(joystick),
    m_pcUp(kMaxArmMotorEncoderValue, kControlStartDelta, kTolerance, kMinPower, kMaxPower), 
    m_pcDown(kMinArmMotorEncoderValue, kControlStartDelta, kTolerance, kMinPower, kMaxPower) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climbSub});
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
  double e = m_climbSub->getArmMotorEncoder();
  frc::SmartDashboard::PutNumber("Climb Winch Encoder", e);

  bool isShiftDownPressed = (m_climbSub->getOperatorShiftState(m_joystick) == DpadConstants::kDown);
  
  // Apply power
  if (isShiftDownPressed) {
    m_climbSub->setWinchPower(pRaw);
  } else {
    double pProportional;
    if (pRaw < 0) {
      pProportional = m_pcDown.getPower(e);
    } else {
      pProportional = m_pcUp.getPower(e);
    }
    m_climbSub->setWinchPower(pProportional);  
  }
  // else if ((pRaw > 0) && (e >= ClimbConstants::kMaxArmMotorEncoderValue)) {
  //   m_climbSub->setWinchPower(0.0);
  // }
  // else if (pRaw > 0 && e >= ClimbConstants::kMaxArmMotorEncoderValue - 10) {
  //   m_climbSub->setWinchPower(pRaw/(e/3));
  // }
  // else if ((pRaw < 0) && (e <= 0)) {
  //   m_climbSub->setWinchPower(0.0);
  // }
  // else if (pRaw < 0 && e <= 10) {
  //   m_climbSub->setWinchPower(pRaw/(e/3));
  // }
  // else {
  //   m_climbSub->setWinchPower(pRaw);
  // }
}

// Relying on command inrerruption to end this
void ClimbWinchCmd::End(bool interrupted) {
}

// Returns true when the command should end.
bool ClimbWinchCmd::IsFinished() { return false; }
