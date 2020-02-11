/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShootCmd.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>

constexpr double kP = 0.0001;
constexpr double kD = 2.0;
constexpr double kSpeedTolerance = 30;
constexpr double kMaxRPM = 21750;
constexpr double kMeasuredTargetSpeed = 15030;


ShootCmd::ShootCmd(ShooterSub* shooterSub, IntakeSub* intakeSub) : m_shooterSub(shooterSub), m_intakeSub(intakeSub) {

  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooterSub});
  AddRequirements({intakeSub});
}

// Called when the command is initially scheduled.
void ShootCmd::Initialize() {
  m_intakeSub->setMagazineIntakePower(-1.0);
  m_shooterSub->setFeedSpeed(1.0);
  m_targetSpeed = kMeasuredTargetSpeed;
  m_lastDiff = 0.0; 
  m_lastTime = frc::RobotController::GetFPGATime();
}


void ShootCmd::Execute() {
  double currentDiff = m_targetSpeed - m_shooterSub->getSpeed();
  double feed = m_targetSpeed / kMaxRPM;
  uint64_t currentTime = frc::RobotController::GetFPGATime();
  double speedDiff = (currentDiff - m_lastDiff)/(currentTime - m_lastTime);
  double speed = (currentDiff*kP) + (speedDiff*kD) + feed;
  printf ("- speedDiff*kD=%f ; currentD=%f ; timediff=%llu ; lastD=%f ;\n" , speedDiff*kD, currentDiff, currentTime - m_lastTime, m_lastDiff);
  m_shooterSub->setSpeed(speed);
  m_lastDiff = currentDiff;
  m_lastTime = currentTime;

 
  // We need to change the target speed based on how close the target is (using the y value on limelight)
   /* if(index < 5){
    double diff = m_targetSpeed - m_shooterSub->getSpeed();
    double feed = m_targetSpeed / kMaxRPM;
  //kP, kSpeedTolerance, and kMaxRPM are arbitrary values for now.
    double speed=diff*kP+feed;
    m_shooterSub->setSpeed(speed);
    if(m_shooterSub->getSpeed() >= m_targetSpeed-kSpeedTolerance and m_shooterSub->getSpeed() <= m_targetSpeed+kSpeedTolerance){
      powers[index]=speed;
      index+=1;
    }
    else {
      index=0;
    }
  }
  else {
    double sum=0;
    for(int i=0; i < 5; i++) {
      sum+=powers[i];
    }
    double avg=sum/5.0;
    m_shooterSub->setSpeed(avg);
  }*/
}

// Called once the command ends or is interrupted.
void ShootCmd::End(bool interrupted) {
    m_shooterSub->setSpeed(0);
    m_intakeSub->setMagazineIntakePower(0);
    m_shooterSub->setFeedSpeed(0);
}
