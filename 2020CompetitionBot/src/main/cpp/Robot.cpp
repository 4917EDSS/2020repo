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
#include <frc/RobotController.h>

void Robot::RobotInit() {
  frc::SmartDashboard::PutString("Target Colour", "____");
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
int underTwentyCounter = 0;
void Robot::RobotPeriodic() { 
 
  uint64_t currentTimeStart = frc::RobotController::GetFPGATime();
  frc2::CommandScheduler::GetInstance().Run(); 
  uint64_t currentTimeFinal = frc::RobotController::GetFPGATime();

  // Count number of loop overruns
  if (currentTimeFinal-currentTimeStart >= 20000){
    std::cout << "cycles since last 20000: " << underTwentyCounter << std::endl;
    std::cout << "diffrence: " << currentTimeFinal - currentTimeStart << std::endl;
    underTwentyCounter = 0;
  }
  else if (currentTimeFinal - currentTimeStart < 20000){
    underTwentyCounter++;
  }
}

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
  m_container.initSubsystems();
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
  //This is an example of how to create a boolean box with specific colours:
  // NetworkTableEntry myBoolean = Shuffleboard.getTab("SmartDashboard")
  //         .add("Target Colour", false)
  //         .withWidget("Boolean Box")
  //         .withProperties(Map.of("colourWhenTrue", "blue", "colourWhenFalse",       
  //         "gray")) .getEntry();

  std::string gameData;
  gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  if (gameData.length() > 0) {
    switch (gameData[0]) {
      case 'B': 
        frc::SmartDashboard::PutString("Target Colour", "Blue");
        targetColour = 'B';
        break;
      case 'R': 
        frc::SmartDashboard::PutString("Target Colour", "Red");
         targetColour = 'R';
        break;
      case 'G': 
        frc::SmartDashboard::PutString("Target Colour", "Green");
        targetColour = 'G';
        break;
      case 'Y':
        frc::SmartDashboard::PutString("Target Colour", "Yellow");
         targetColour = 'Y';
        break;
      default:
        frc::SmartDashboard::PutString("Target Colour", "???");
        break;
    }
  }
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

