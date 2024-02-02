// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#define TURBO_ENABLED 1

void Robot::RobotInit() {
  m_leftMotor1.AddFollower(m_leftMotor2);
  m_rightMotor1.AddFollower(m_rightMotor2);

  m_rightMotor1.SetInverted(true);
}

void Robot::RobotPeriodic() {
}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  Drive();
  RunConveyor();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

void Robot::RunConveyor(){
  if (opController.GetRawAxis(2)){
    m_conveyorMotor.Set(opController.GetRawAxis(2) * 0.3);
  } else if (opController.GetRawAxis(3)){
    m_conveyorMotor.Set(-opController.GetRawAxis(3) * 0.3);
  } else{
    m_conveyorMotor.Set(0);
  }
}

void Robot::Drive(){
  double leftJoystickValue = driveController.GetRawAxis(1);
  double rightJoystickValue = driveController.GetRawAxis(4);

  // Normalize input to have hypotenuse of 1
  double var = sqrt((1/(pow(leftJoystickValue, 2) + pow(rightJoystickValue, 2))));
  
  double simulatedX = leftJoystickValue * var *
                        ((leftJoystickValue > 0) ? 1 : -1);

  double simulatedY = rightJoystickValue * var *
                        ((rightJoystickValue > 0) ? 1 : -1);
  

  #if TURBO_ENABLED 
    double boostEnabled = driveController.GetRawAxis(3);
    if (boostEnabled) {
      m_robotDrive.ArcadeDrive(simulatedX*(0.5 + 0.3 * driveController.GetRawAxis(3)), simulatedY/2*(0.5 + 0.3 * driveController.GetRawAxis(3)));
    } else {
      m_robotDrive.ArcadeDrive(simulatedX*0.5, simulatedY*0.5);
    }
  #else             
    m_robotDrive.ArcadeDrive(simulatedX*0.5, simulatedY*0.5);
  #endif            
}

// void Robot::RunClimber(double speed){
//   if (abs(speed) < 0.05){
//     speed = 0;
//   }

//   m_climbMotorLeft.Set(speed);
//   m_climbMotorRight.Set(speed);
// }

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif