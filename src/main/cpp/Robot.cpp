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

  m_armEncoder.SetDistancePerPulse(1.0 / 2048);
  m_armEncoder.Reset();
  m_armController.SetIZone(0.015);

  m_rightMotor1.SetInverted(true);

  rightSlammer.SetInverted(false);
  leftSlammer.SetInverted(true);
}

void Robot::RobotPeriodic() {

}

// AUTON  ==================================================================
void Robot::AutonomousInit() {
  startTime = frc::Timer::GetFPGATimestamp();
}

void Robot::AutonomousPeriodic() {
  units::second_t currentTime = frc::Timer::GetFPGATimestamp();
  double elapsedTime = currentTime.value() - startTime.value();

  if (elapsedTime < 2.5){
    Shoot(startTime);
  } else {
    shootMotor1.Set(0);
    shootMotor2.Set(0);
  }

  if (elapsedTime < 6.5 && elapsedTime > 2.5){
    Drive(0.5, 0, false);
  } else {
    Drive(0, 0, false);
  }

}

// TELEOP ==================================================================
void Robot::TeleopInit() {
  shootMotor1.Set(0);
  shootMotor2.Set(0);

  isShooting = false;
}

void Robot::TeleopPeriodic() {
  // DRIVE ====================================================================
  Drive( driveController.GetRawAxis(3) - driveController.GetRawAxis(2), 
    driveController.GetRawAxis(0), 
    driveController.GetRawButton(1));

  // ARM ====================================================================
  if (opController.GetRawButtonPressed(1)){
    setpoint = 0;
  } else if (opController.GetRawButtonPressed(4)){
    // amp
    setpoint = 0.129;
  } else if (opController.GetRawButtonPressed(2)){
    // akimbo 
    setpoint = 0.267;
  } else if (opController.GetRawButtonPressed(3)){
    // intake
    setpoint = 0.058;
  }

  if (abs(opController.GetRawAxis(1)) > 0.01){
    setpoint = m_armEncoder.GetDistance();
    doPid = false;
  }else 
    doPid = true;

  RotateArm(opController.GetRawAxis(1));

  if (opController.GetRawButton(9) && opController.GetRawButton(10)){
    setpoint = 0;
    m_armEncoder.Reset();
  }

   // MECHANISM ==================================================================== 
  if (opController.GetRawAxis(2) == 1 && opController.GetRawAxis(3) == 1 && !isShooting){
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
  frc::SmartDashboard::PutNumber("Current Setpoint", setpoint);
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
  if (doPid){
    double output = m_armController.Calculate(m_armEncoder.GetDistance(), setpoint);
    if (output > 1){
      output = 1;
    } else if (output < -1){
      output = -1;
    }
    rightSlammer.Set(output);
    leftSlammer.Set(output);
    } else {
      if (abs(speed) < 0.1){
        rightSlammer.Set(0);
        leftSlammer.Set(0);
        return;
      }
      rightSlammer.Set(speed);
      leftSlammer.Set(speed);
  }
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