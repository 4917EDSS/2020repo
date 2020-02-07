/*----------------------------------------------------------------------------*/
/* cpyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShootCmd.h"

constexpr double kP=0.001;
constexpr double kSpeedTolerance=10;
constexpr double kMaxRPM=5000;


ShootCmd::ShootCmd(ShooterSub* shooterSub, IntakeSub* intakeSub, double targetspeed) : m_shooterSub(shooterSub), m_intakeSub(intakeSub), targetspeed(targetspeed){

  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooterSub});
  AddRequirements({intakeSub});
}

// Called when the command is initially scheduled.
void ShootCmd::Initialize() {
  m_shooterSub->setSpeed(0.3);
  m_intakeSub->setIntake(-1.0);
  m_shooterSub->setFeedSpeed(1.0);
}

void ShootCmd::Execute() {
  //kP, kSpeedTolerance, and kMaxRPM are arbitrary values for now.
  if(index < 5){
    double diff = targetspeed - m_shooterSub->getSpeed();
    double feed = targetspeed / kMaxRPM;
    double speed=diff*kP+feed;
    m_shooterSub->setSpeed(speed);
    if(m_shooterSub->getSpeed() >= targetspeed-kSpeedTolerance and m_shooterSub->getSpeed() <= targetspeed+kSpeedTolerance){
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
  }
}

// Called once the command ends or is interrupted.
void ShootCmd::End(bool interrupted) {
    m_shooterSub->setSpeed(0);
    m_intakeSub->setIntake(0);
    m_shooterSub->setFeedSpeed(0);
}
