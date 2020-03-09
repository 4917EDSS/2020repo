/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/VisionAlignmentCmd.h"
#include <frc/RobotController.h>

constexpr double kMaxEndVelocity = 0.1;  // In degrees per iteration

VisionAlignmentCmd::VisionAlignmentCmd(VisionSub* visionSub, DrivetrainSub* drivetrainSub, bool isFar)
  : m_visionSub(visionSub),
    m_drivetrainSub(drivetrainSub),
    m_isFar(isFar),
    m_lastX(0),
    m_pController(0, 20, VisionConstants::kXAllignmentTolerence, 0.10, 0.2) {
  
  AddRequirements({visionSub, drivetrainSub});
}

// Called when the command is initially scheduled.
void VisionAlignmentCmd::Initialize() {
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
  std::cout << frc::RobotController::GetFPGATime() - m_startTime << std::endl;
  double x = m_visionSub->getVisionTarget();
  double outputPower = m_pController.getPower(x);
  m_drivetrainSub->tankDrive(-outputPower, outputPower);
}

// Called once the command ends or is interrupted.
void VisionAlignmentCmd::End(bool interrupted) {
  m_drivetrainSub->tankDrive(0.0, 0.0);
  m_visionSub->setNeutralVisionPipeline();
}

// Returns true when the command should end.
bool VisionAlignmentCmd::IsFinished() { 
  if((frc::RobotController::GetFPGATime() - m_startTime) < 1500000) {
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
