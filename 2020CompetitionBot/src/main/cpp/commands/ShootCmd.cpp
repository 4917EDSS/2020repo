/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include "Constants.h"
#include "commands/ShootCmd.h"

constexpr double kP = 0.00005;
constexpr double kI = 0.0002;
constexpr double kD = 0.0;
constexpr double kSpeedTolerance = 110.0;

ShootCmd::ShootCmd(ShooterSub* shooterSub, IntakeSub* intakeSub, bool isFar) 
  : m_shooterSub(shooterSub), 
    m_intakeSub(intakeSub),
    m_isFar(isFar)
  {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooterSub, intakeSub});
}

// Called when the command is initially scheduled.
void ShootCmd::Initialize() {
  m_intakeSub->setFrontRollerIntakePower(1.0);
  m_targetSpeed = (m_isFar ? ShooterConstants::kFarTargetSpeed : ShooterConstants::kCloseTargetSpeed);
  m_lastDiff = 0.0; 
  m_lastTime = frc::RobotController::GetFPGATime();
  m_integralDiff = 0.0;
  m_index = 0;
}

double ShootCmd::runPID(){
  double currentDiff = m_targetSpeed - m_shooterSub->getSpeed();
  double feed = m_targetSpeed / ShooterConstants::kMaxRPM;
  uint64_t currentTime = frc::RobotController::GetFPGATime();
  double timeSinceLast = static_cast<double>(currentTime - m_lastTime)/1000000.0;
  double speedDiff = (currentDiff - m_lastDiff)/timeSinceLast;
  // don't want to start integrating while getting up to speed
  if(currentDiff < 1000){
    double addedIntegralDiff = (timeSinceLast)*((currentDiff + m_lastDiff)/2.0);
    m_integralDiff += addedIntegralDiff;
    std::cout << m_integralDiff  << "***\n";
  }
  m_lastDiff = currentDiff;
  m_lastTime = currentTime;
  return (currentDiff*kP) + (m_integralDiff*kI) + (speedDiff*kD)  + feed;
}

void ShootCmd::Execute() {
  m_shooterSub->setSpeed(runPID());
  m_intakeSub->setMagazineIntakePower(-0.75);//\this is just for testing what constant speed is needed to allow for a constant movement of the shooter
  // double currentDiff = m_targetSpeed - m_shooterSub->getSpeed();
  // frc::SmartDashboard::PutNumber("currentDiff", currentDiff);
  // if (fabs(currentDiff) <= kSpeedTolerance) {
  //   m_intakeSub->setMagazineIntakePower(-0.75);
  // }
  // else{
  //   m_intakeSub->setMagazineIntakePower(0.0);
  // }


  // // We need to change the target speed based on how close the target is (using the y value on limelight)
  // if(m_index < 5){
  //   double speed = runPID();
  //   m_shooterSub->setSpeed(speed);

  //   if(fabs(currentDiff) <= kSpeedTolerance){
  //     powers[m_index]=speed;
  //     std::cout << "good " << speed << "\n";
  //     m_index+=1;
  //   }
  //   else {
  //     m_index=0;
  //     std::cout << speed << "\n";
  //   }
  // }
  // else {
  //   double sum=0;
  //   for(int i=0; i < 5; i++) {
  //     sum+=powers[i];
  //   }
  //   double avg=sum/5.0;
  //   m_shooterSub->setSpeed(avg);
  // }
}

// Called once the command ends or is interrupted.
void ShootCmd::End(bool interrupted) {
    m_shooterSub->setSpeed(0);
    m_intakeSub->setMagazineIntakePower(0);
    m_intakeSub->setFrontRollerIntakePower(0);
}
