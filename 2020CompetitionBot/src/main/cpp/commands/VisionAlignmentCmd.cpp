/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/VisionAlignmentCmd.h"
#include "subsystems/VisionSub.h"

constexpr double kP = 0.5;

VisionAlignmentCmd::VisionAlignmentCmd(VisionSub* visionSub, DrivetrainSub* drivetrainSub, bool isFar) :
  m_visionSub(visionSub),
  m_drivetrainSub(drivetrainSub),
  m_isFar(isFar)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({visionSub});
  AddRequirements({drivetrainSub});
}

// Called when the command is initially scheduled.

void VisionAlignmentCmd::Initialize() {
  m_drivetrainSub->shiftDown();
  m_isAligned = false;
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

  frc::SmartDashboard::PutNumber("FrontVisionTargetX", x);
  double power = (x / VisionConstants::kXMax) * kP;
  if (x > VisionConstants::kXAllignmentTolerence || x < -(VisionConstants::kXAllignmentTolerence)) { 
      m_drivetrainSub->tankDriveVolts((-power), (power));

  } else {
    m_isAligned = true;
    m_drivetrainSub->tankDriveVolts(0.0, 0.0);
  }
}

// Called once the command ends or is interrupted.
void VisionAlignmentCmd::End(bool interrupted) {
  m_drivetrainSub->tankDriveVolts(0.0, 0.0);
}

// Returns true when the command should end.
bool VisionAlignmentCmd::IsFinished() { 
 return m_isAligned;
}
