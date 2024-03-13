// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <cameraserver/CameraServer.h>
#include <frc/TimedRobot.h>

// ROBOT ===================================================================
void Robot::RobotInit() {
  m_leftMotor1.AddFollower(m_leftMotor2);
  m_rightMotor1.AddFollower(m_rightMotor2);
  shootMotor1.Follow(shootMotor2);

  m_rightMotor1.SetInverted(true);
  rightSlammer.SetInverted(true);
}

void Robot::RobotPeriodic() {
}

// AUTON  ==================================================================
void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {
}

// TELEOP ==================================================================
void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
  Drive( driveController.GetRawAxis(3) - driveController.GetRawAxis(2), 
    driveController.GetRawAxis(0), 
    driveController.GetRawButton(1));
  RotateArm(opController.GetRawAxis(1));
  RunShooter(opController.GetRawAxis(3) - opController.GetRawAxis(2), opController.GetRawButton(1));
}

void Robot::Drive(double xSpeed, double zRotation, bool turnInPlace){
  if (xSpeed < 0){
    zRotation = -zRotation;
  }

  if (xSpeed > 0){
      if (turnInPlace)
          m_robotDrive.CurvatureDrive(-filter.Calculate(xSpeed), zRotation, turnInPlace);
      else 
          m_robotDrive.CurvatureDrive(-filter.Calculate(xSpeed), -zRotation, turnInPlace);
  } else {
      m_robotDrive.CurvatureDrive(-filter.Calculate(xSpeed), zRotation, turnInPlace);
  }
}

void Robot::RotateArm(double speed){
  rightSlammer.Set(speed);
  leftSlammer.Set(speed);
}

void Robot::RunShooter(double speed, bool shoot){
  if (shoot){
    shootMotor1.Set(-1);
  } else
    shootMotor1.Set(speed * 0.1);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif