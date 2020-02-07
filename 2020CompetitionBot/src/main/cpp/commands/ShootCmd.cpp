/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShootCmd.h"

constexpr double P=0.001;
constexpr double MAX_RPM=5000;
constexpr double kMeasuredTargetSpeed = 3000;


ShootCmd::ShootCmd(ShooterSub* shooterSub, IntakeSub* intakeSub) : m_shooterSub(shooterSub), m_intakeSub(intakeSub) {

  // Use addRequirements() here to declare subsystem dependencies.
AddRequirements({shooterSub});
AddRequirements({intakeSub});
}

// Called when the command is initially scheduled.
void ShootCmd::Initialize() {
  m_shooterSub->setSpeed(0.3);
  m_intakeSub->setIntake(-1.0);
  m_shooterSub->setFeedSpeed(1.0);
  m_targetSpeed = kMeasuredTargetSpeed;
}

  void ShootCmd::Execute() {      
    // We need to change the target speed based on how close the target is (using the y value on limelight)
    double diff = m_targetSpeed - m_shooterSub->getSpeed();
    double feed = m_targetSpeed / MAX_RPM;
     m_shooterSub->setSpeed(diff * P + feed);
    //P and MAX_RPM are arbitrary values for now.
}

// Called once the command ends or is interrupted.
void ShootCmd::End(bool interrupted) {
    m_shooterSub->setSpeed(0);
    m_intakeSub->setIntake(0);
    m_shooterSub->setFeedSpeed(0);
}
