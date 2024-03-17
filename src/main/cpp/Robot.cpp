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
  // shootMotor2.Follow(shootMotor1);

  m_armEncoder.SetDistancePerPulse(1.0 / 2048);
  m_armEncoder.Reset();

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
  shootMotor1.Set(0);
  shootMotor2.Set(0);
}

void Robot::TeleopPeriodic() {
  // DRIVE ====================================================================
  Drive( driveController.GetRawAxis(3) - driveController.GetRawAxis(2), 
    driveController.GetRawAxis(0), 
    driveController.GetRawButton(1));

  // ARM ====================================================================
  // if (opController.GetRawButton(5)){
  //   setpoint = 0;
  // } else if (opController.GetRawButton(6)){
  //   setpoint = 0.5;
  // } else if (opController.GetRawButton(7)){
  //   setpoint = -0.1;
  // } else if (opController.GetRawButton(8)){
  //   setpoint = 0.3;
  // }
  RotateArm(opController.GetRawAxis(1));

   // MECHANISM ==================================================================== 
  if (opController.GetRawButtonPressed(1)){
    isShooting = true;
    startTime = frc::Timer::GetFPGATimestamp();
  }

  if (isShooting){
    Shoot(startTime);
  } else {
    RunShooter(opController.GetRawAxis(3) - opController.GetRawAxis(2));
  }

  // TELEMETRY ====================================================================
  frc::SmartDashboard::PutNumber("Arm Angle", m_armEncoder.GetDistance());
}

void Robot::Drive(double xSpeed, double zRotation, bool turnInPlace){
  if (xSpeed < 0){
    zRotation = -zRotation;
  }

  if (xSpeed > 0){
      if (turnInPlace)
          m_robotDrive.CurvatureDrive(filter.Calculate(xSpeed), zRotation/2, turnInPlace);
      else 
          m_robotDrive.CurvatureDrive(-filter.Calculate(xSpeed), -zRotation, turnInPlace);
  } else {
      m_robotDrive.CurvatureDrive(-filter.Calculate(xSpeed), -zRotation, turnInPlace);
  }
}

void Robot::RotateArm(double speed){
  if (abs(speed) < 0.1){
    rightSlammer.Set(0);
    leftSlammer.Set(0);
    return;
  }
  rightSlammer.Set(speed);
  leftSlammer.Set(speed);

  // PID
  // rightSlammer.Set(m_armController.Calculate(m_armEncoder.GetDistance(), setpoint));
}

void Robot::Shoot(units::second_t startTime){

  units::second_t currentTime = frc::Timer::GetFPGATimestamp();
  double elapsedTime = currentTime.value() - startTime.value();

  if (elapsedTime < 0.5){
    shootMotor2.Set(-1);
  } else if (elapsedTime < 1.5){
    shootMotor2.Set(-1);
    shootMotor1.Set(1);
  } else {
    shootMotor1.Set(0);
    shootMotor2.Set(0);
    isShooting = false;
  }

}

void Robot::RunShooter(double speed){
   shootMotor1.Set(speed * 0.1);
   shootMotor2.Set(speed * -0.1);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif