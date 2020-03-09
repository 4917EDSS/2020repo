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

// ----------------------------------------------------------------------------------------
// This was added for the proportional controller approach. We abandoned it.
// ----------------------------------------------------------------------------------------
// constexpr double kTolerance = 2.0;
// constexpr double kControlStartDelta = 10.0;
// constexpr double kMinPower = 0.05;
// constexpr double kMaxPower = 1.0;
// ----------------------------------------------------------------------------------------
constexpr double kMaxArmMotorEncoderValue = 471.0;
constexpr double kMinArmMotorEncoderValue = 10.0; // WARNING: This must be > 0.
                                                  // The string can wrap around itself different
                                                  // ways when winding up and result in bottoming
                                                  // out the climb arm and the encoder value can
                                                  // be a value like 5.0 when bottomed out. The
                                                  // motor is strong enough to break the string,
                                                  // so once we've extended the arm and retracted
                                                  // it all the way, we need to stop retraction
                                                  // safely before bottoming out when encoder
                                                  // reports a value close to but > 5.0. The value
                                                  // 10.0 was chosen. 10.0 encoder ticks equates
                                                  // to about 0.75 inches of arm travel. This will
                                                  // allow the arm to retract to anywhere from nearly
                                                  // bottoming out no closer then 0.75 inches from
                                                  // bottomig out depending on how the string winds
                                                  // on itself.

ClimbWinchCmd::ClimbWinchCmd(ClimberSub* climberSub, frc::Joystick* joystick)
  : m_climberSub(climberSub),
    m_joystick(joystick) {
  
  AddRequirements({climberSub});
}

// Called when the command is initially scheduled.
// We just turn the power on and leave it until command is interrupted calling End
void ClimbWinchCmd::Initialize() {
  
}

// Called repeatedly when this Command is scheduled to run
void ClimbWinchCmd::Execute() {
  
  double power = -1 * (m_joystick->GetThrottle());
  // Accomodate drift on controller joystick by ignoring tiny values.

  if(fabs(power) < 0.1) {
    power = 0;
  }

  double e = m_climberSub->getArmMotorEncoder();

  bool isShiftDownPressed = (m_climberSub->getOperatorShiftState(m_joystick) == DpadConstants::kDown);

  // Apply power
  if (isShiftDownPressed) {
    m_climberSub->setWinchPower(power);
  }
  else if ((power > 0) && (e >= kMaxArmMotorEncoderValue)) {
    m_climberSub->setWinchPower(0.0);
  }
  else if (power > 0 && e >= kMaxArmMotorEncoderValue - 10) {
    m_climberSub->setWinchPower(power / (11 - (kMaxArmMotorEncoderValue - e)));
  }
  else if ((power < 0) && (e <= 0)) {
    m_climberSub->setWinchPower(0.0);
  }
  else if (power < 0 && e <= 10) {
    m_climberSub->setWinchPower(power / (11 - (kMaxArmMotorEncoderValue - e)));
  }
  else {
    m_climberSub->setWinchPower(power);
  }

}

// Relying on command inrerruption to end this
void ClimbWinchCmd::End(bool interrupted) {
  // Joystick-controller commands don't usually have an End
}

// Returns true when the command should end.
bool ClimbWinchCmd::IsFinished() { return false; }
