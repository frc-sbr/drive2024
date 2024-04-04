// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <cameraserver/CameraServer.h>
#include <frc/TimedRobot.h>

// ROBOT ===================================================================
void Robot::RobotInit() {
  // Setting up drive motors
  m_leftMotor1.AddFollower(m_leftMotor2);
  m_rightMotor1.AddFollower(m_rightMotor2);

  m_leftMotor1.SetInverted(true);
  m_rightMotor1.SetInverted(false);

  // Setting up robot Arm
  m_armController.SetIZone(0.015);

  rightSlammer.SetInverted(false);
  leftSlammer.SetInverted(true); 

  frc::CameraServer::StartAutomaticCapture();
}

void Robot::RobotPeriodic() { 

}

// AUTON  ==================================================================
void Robot::AutonomousInit() {
  // setting initial conditions

  //start time
  startTime = frc::Timer::GetFPGATimestamp();

  //shoot angle & pid usage
  setpoint = 0.267;
  doPid = true;
}

void Robot::AutonomousPeriodic() {
  double elapsedTime = frc::Timer::GetFPGATimestamp().value() - startTime.value();

  if (elapsedTime < 3){

    // get arm into correct angle
    RotateArm(0);
  } else if (elapsedTime < 5.5){

    //shoot for 2.5 seconds
    rightSlammer.Set(0);
    leftSlammer.Set(0);
    Shoot(startTime + 3_s);
  } else if (elapsedTime < 7){
    
    // taxi for 4 seconds and rotate arm into ground mode
    setpoint = 0;
    shootMotor1.Set(0);
    shootMotor2.Set(0);
    Drive(0.5, 0, false);
    RotateArm(0);
  } else {

    // stop
    Drive(0, 0, false);
  }

  frc::SmartDashboard::PutNumber("Current Setpoint", setpoint);

}

// TELEOP ==================================================================
void Robot::TeleopInit() {

  //set all motors to 0
  shootMotor1.Set(0);
  shootMotor2.Set(0);
  rightSlammer.Set(0);
  leftSlammer.Set(0);

  isShooting = false;
  doPid = true;
}

void Robot::TeleopPeriodic() {
  // DRIVE ====================================================================
  Drive( driveController.GetRawAxis(3) - driveController.GetRawAxis(2), 
    driveController.GetRawAxis(0), 
    driveController.GetRawButton(1));

  // ARM ====================================================================
  if (opController.GetRawButtonPressed(1)){
    // ground
    setpoint = 0;
  } else if (opController.GetRawButtonPressed(4)){
    // amp
    setpoint = 0.250;
  } else if (opController.GetRawButtonPressed(2)){
    // akimbo 
    setpoint = 0.267;
  } else if (opController.GetRawButtonPressed(3)){
    // intake
    setpoint = 0.055;
  }

  // if the joystick is moved, disable pid
  if (abs(opController.GetRawAxis(1)) > 0.04){
    doPid = false;
  }else 
    doPid = true;

  RotateArm(opController.GetRawAxis(1));

  // if both joystick buttons are pressed, reset encoder
  if (opController.GetRawButton(9) && opController.GetRawButton(10)){
    setpoint = 0;
  }

   // MECHANISM ====================================================================

   // if both triggers are pressed for the first time, begin shooting 
  if (opController.GetRawButtonPressed(6)){
    isShooting = true;
    startTime = frc::Timer::GetFPGATimestamp();
  }

  // triggers are disabled during shoot
  if (isShooting){
    Shoot(startTime);
  } else {
    RunShooter(opController.GetRawAxis(3) - opController.GetRawAxis(2));
  }

  // TELEMETRY ====================================================================
  frc::SmartDashboard::PutNumber("Current Setpoint", setpoint);
}

void Robot::Drive(double xSpeed, double zRotation, bool turnInPlace){
  if (xSpeed < 0){
    zRotation = -zRotation;
  }

  if (xSpeed > 0){
      if (turnInPlace)
          m_robotDrive.CurvatureDrive(filter.Calculate(xSpeed), -zRotation/3, turnInPlace);
      else 
          m_robotDrive.CurvatureDrive(-filter.Calculate(xSpeed), zRotation/1.5, turnInPlace);
  } else {
    if (turnInPlace){
      m_robotDrive.CurvatureDrive(-filter.Calculate(xSpeed), zRotation/3, turnInPlace);
    } else {
      m_robotDrive.CurvatureDrive(-filter.Calculate(xSpeed), zRotation/1.5, turnInPlace);
    }
      
  }
}

void Robot::RotateArm(double speed){
  // set the joystick output, move the setpoint with the arm
  if (abs(speed) < 0.1){
    rightSlammer.Set(0);
    leftSlammer.Set(0);
    return;
  }
  rightSlammer.Set(speed);
  leftSlammer.Set(speed);
}

void Robot::Shoot(units::second_t startTime){
  double elapsedTime = frc::Timer::GetFPGATimestamp().value() - startTime.value();

  if (elapsedTime < 0.5){

    // rotate bottom wheel
    shootMotor2.Set(-1);
  } else if (elapsedTime < 1.5){

    // rotate both wheels
    shootMotor2.Set(-1);
    shootMotor1.Set(1);
  } else {

    // stop 
    shootMotor1.Set(0);
    shootMotor2.Set(0);
    isShooting = false;
  }

}

void Robot::RunShooter(double speed){
   shootMotor1.Set(speed * 0.15);
   shootMotor2.Set(speed * -0.15);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif