/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/DriverStation.h>
#include <cstdlib>


void Robot::RobotInit() {



}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.getAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
  
}
/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {

std::string gameData;
gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  if (gameData.length() > 0){
    switch (gameData[0]) {
      case 'B': 
        frc::SmartDashboard::PutString("Target Colour", "Blue");
        break;
      case 'R': 
        frc::SmartDashboard::PutString("Target Colour", "Red");
        break;
      case 'G': 
        frc::SmartDashboard::PutString("Target Colour", "Green");
        break;
      case 'Y':
        frc::SmartDashboard::PutString("Target Colour", "Yellow");
        break;
      default:
        frc::SmartDashboard::PutString("Target Colour", "____");
        break;
    }
  }
  else {
    frc::SmartDashboard::PutString("Target Colour", "Transmission not received");
  }
}
/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
