/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/VisionAlignmentCmd.h"
#include <frc/RobotController.h>

constexpr double kP = 0.05;
constexpr double kMaxPower = 0.5;
constexpr double kMinPower = kMinimumTurningPower;
constexpr double kMaxEndVelocity = 0.1;  // In degrees per iteration 

VisionAlignmentCmd::VisionAlignmentCmd(VisionSub* visionSub, DrivetrainSub* drivetrainSub, bool isFar) :
  m_visionSub(visionSub),
  m_drivetrainSub(drivetrainSub),
  m_isFar(isFar),
  m_lastX(0)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({visionSub, drivetrainSub});
}

// Called when the command is initially scheduled.

void VisionAlignmentCmd::Initialize() {
  printf("vision started");
  m_startTime = frc::RobotController::GetFPGATime();
  m_lastX = 0;
  m_drivetrainSub->shiftDown();
  if(m_isFar) {
    m_visionSub->setFarVisionPipeline();
  }
  else {
    m_visionSub->setShortVisionPipeline();
  }
}

// Called repeatedly when this Command is scheduled to run
void VisionAlignmentCmd::Execute() {
  double x = m_visionSub->getVisionTarget();
  bool isNegative = false;

  frc::SmartDashboard::PutNumber("FrontVisionTargetX", x);
  
  // Get rid of negative to facilitate math.  We'll add it back at the end.
  if(x < 0) {
    isNegative = true;
    x *= -1.0;  
  }

  //turn proportionally 
  double power = (x / VisionConstants::kXMax) * kP;
  power += kMinPower;
  if(power > kMaxPower) {
    power = kMaxPower;
  }

  //adds back negative
  if(isNegative) {
    power *= -1.0;  
  }
  frc::SmartDashboard::PutNumber("LPower", power);
  printf("vision x=%f power=%f\n", x, power);
  if(fabs(x) > VisionConstants::kXAllignmentTolerence)
  { 
    m_drivetrainSub->tankDrive((-power), (power));
  } else {
    m_drivetrainSub->tankDrive(0.0, 0.0);
  }
}

// Called once the command ends or is interrupted.
void VisionAlignmentCmd::End(bool interrupted) {
  m_drivetrainSub->tankDrive(0.0, 0.0);
  printf("vision ended");
  m_visionSub->setNeutralVisionPipeline();
}

// Returns true when the command should end.
bool VisionAlignmentCmd::IsFinished() { 
  if((frc::RobotController::GetFPGATime() - m_startTime) < 500000) {
    return false;
  }
  double x = m_visionSub->getVisionTarget();
  double currentVelocity = (x - m_lastX);
  m_lastX = x;
  if((fabs(x) < VisionConstants::kXAllignmentTolerence) && (fabs(currentVelocity) < kMaxEndVelocity)) {
    return true;
  }
  else {
    return false;
  }
}
