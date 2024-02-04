// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#define TURBO_ENABLED 0

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("Left Encoder", m_robotSubsystem.GetLeftEncoder());
  frc::SmartDashboard::PutNumber("Right Encoder", m_robotSubsystem.GetRightEncoder());
}

void Robot::AutonomousInit() {
  m_robotSubsystem.ResetEncoders();
}

void Robot::AutonomousPeriodic() {
  if (m_robotSubsystem.GetLeftEncoder() < 0.3){
    m_robotSubsystem.JoystickDrive(0.2, 0, false);
  } else {
    m_robotSubsystem.JoystickDrive(0, 0, false);
  }
}

void Robot::TeleopInit() {
  m_robotSubsystem.ResetEncoders();
}

void Robot::TeleopPeriodic() {
  #if TURBO_ENABLED
    m_robotSubsystem.JoystickDrive(
      driveController.GetRawAxis(3) - driveController.GetRawAxis(2), 
      driveController.GetRawAxis(0), 
      driveController.GetRawButton(1));
  #else 
    m_robotSubsystem.JoystickDrive(
      (driveController.GetRawAxis(3) - driveController.GetRawAxis(2))/2, 
      driveController.GetRawAxis(0), 
      driveController.GetRawButton(1));
  #endif

  m_robotSubsystem.RunConveyor(opController.GetRawAxis(2) - opController.GetRawAxis(3));
}

// void Robot::RunConveyor(){
//   if (opController.GetRawAxis(2)){
//     m_conveyorMotor.Set(opController.GetRawAxis(2) * 0.3);
//   } else if (opController.GetRawAxis(3)){
//     m_conveyorMotor.Set(-opController.GetRawAxis(3) * 0.3);
//   } else{
//     m_conveyorMotor.Set(0);
//   }
// }

// void Robot::Drive(){
//   double leftJoystickX = driveController.GetRawAxis(0);
//   double leftJoystickY = driveController.GetRawAxis(1);
//   double leftTrigger = driveController.GetRawAxis(2);
//   double rightTrigger = driveController.GetRawAxis(3);
//   double rightJoystickX = driveController.GetRawAxis(4);
//   double rightJoystickY = driveController.GetRawAxis(5);

//   bool turnInPlace = (driveController.GetRawButton(1));

//   #if TURBO_ENABLED 
//     m_robotDrive.CurvatureDrive((rightTrigger - leftTrigger)/3, ((rightTrigger>0) ? 1 : -1)*rightJoystickX, turnInPlace);
//   #else            
//     m_robotDrive.ArcadeDrive(driveController.GetRawAxis(1)/2.0, driveController.GetRawAxis(4)/2.0);
//   #endif
// }

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