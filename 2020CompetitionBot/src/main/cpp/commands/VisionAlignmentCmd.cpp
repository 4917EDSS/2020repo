/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands\VisionAlignmentCmd.h"

VisionAlignmentCmd::VisionAlignmentCmd(VisionSub* visionSub, DrivetrainSub* drivetrainSub) :
  m_visionSub(visionSub),
  m_drivetrainSub(drivetrainSub)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({visionSub});
  AddRequirements({drivetrainSub});
}

// Called when the command is initially scheduled.
void VisionAlignmentCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void VisionAlignmentCmd::Execute() {
  // REVIEW: [Jason] Should use a constant for camera ID
  double x = m_visionSub->getVisionTarget(1);
 
  if (x > VisionConstants::kXAllignmentTolerence || x < -(VisionConstants::kXAllignmentTolerence)) { 
    if (x > VisionConstants::kXMax) {
      x = VisionConstants::kXMax;
    } else if (x < -VisionConstants::kXMax) { 
      x = -VisionConstants::kXMax;
    }
    if (x > 0) {
      // turn right 
      m_drivetrainSub->tankDriveVolts(units::volt_t(x / VisionConstants::kXMax), units::volt_t(-x / VisionConstants::kXMax));

    } else if (x < 0) {
      //turn left
      m_drivetrainSub->tankDriveVolts(units::volt_t(-x / VisionConstants::kXMax), units::volt_t(x / VisionConstants::kXMax));

    }
  }
  // REVIEW: [Jason] Since we already know here we have acheived alignment, perhaps we should have an else block with this:
  //                    - set a flag indicating alignment was acheived
  //                    - set left/right drive volts to 0 (stopping motors - so we don't oscillate from overshots)
}

// Called once the command ends or is interrupted.
void VisionAlignmentCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool VisionAlignmentCmd::IsFinished() { 
  // REVIEW: [Jason] If we set a flag in Execute when alignement is acheived, we can use it here instead of getting and checking x again.
  //                 We *must* make motors stop, and that is best done in Execute so we don't oscillate from overshots.

  // REVIEW: [Jason] Should use a constant for camera ID
  double x = m_visionSub->getVisionTarget(1);

  if (x > VisionConstants::kXAllignmentTolerence || x < -(VisionConstants::kXAllignmentTolerence)) { 
    return true;
  } else {
    return false;
  }
}
