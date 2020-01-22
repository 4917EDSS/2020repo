/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShootCmd.h"

constexpr double P=0.001;
constexpr double MAX_RPM=5000;



ShootCmd::ShootCmd(ShooterSub* subsystem, double targetspeed) : m_ShootSub(subsystem), targetspeed(targetspeed){
  // Use addRequirements() here to declare subsystem dependencies.
AddRequirements({subsystem});
}

// Called when the command is initially scheduled.
void ShootCmd::Initialize() {
  m_ShootSub->setSpeed(0.6);
}
  void ShootCmd::Execute() {
    double diff=targetspeed-m_ShootSub->getSpeed();
    double feed=targetspeed/MAX_RPM;
    m_ShootSub->setSpeed(diff*P+feed);
    //P and MAX_RPM are arbitrary values for now.
}



// Called once the command ends or is interrupted.
void ShootCmd::End(bool interrupted) {
    m_ShootSub->setSpeed(0);
}
