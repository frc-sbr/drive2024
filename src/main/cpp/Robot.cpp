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
  m_armController.SetIZone(0.100);

  rightSlammer.SetInverted(false);
  leftSlammer.SetInverted(true); 

  shootMotor1.SetInverted(true);
  shootMotor2.SetInverted(false);
  shootMotor3.SetInverted(false);
  shootMotor4.SetInverted(true);

  frc::CameraServer::StartAutomaticCapture();

  m_armEncoder.SetPositionConversionFactor(1.0/400 * 2 * M_PI);
  m_armEncoder.SetPosition(0);

  // Setting up auto choosing
  m_chooser.SetDefaultOption(kAutoTaxi, kAutoTaxi);
  m_chooser.AddOption(kAutoShoot, kAutoShoot);
  m_chooser.AddOption(kAutoWait, kAutoWait);
  m_chooser.AddOption(kAutoTaxiShoot, kAutoTaxiShoot);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  frc::CameraServer::StartAutomaticCapture();
}

void Robot::RobotPeriodic() { 

}

// AUTON  ==================================================================
void Robot::AutonomousInit() {
  // setting initial conditions

  m_autoSelected = m_chooser.GetSelected();
  
  //start time
  startTime = frc::Timer::GetFPGATimestamp();

  //shoot angle & pid usage
  doPid = true;
}

void Robot::AutonomousPeriodic() {
  double elapsedTime = frc::Timer::GetFPGATimestamp().value() - startTime.value();

  if (m_autoSelected == kAutoTaxiShoot) {
    setpoint = SHOOT;
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
      // setpoint = ZERO;
      shootMotor1.Set(0);
      shootMotor2.Set(0);
      shootMotor4.Set(0);
      shootMotor3.Set(0);
      Drive(-0.4, 0, false);
      RotateArm(0);
    } else {
      // stop

      rightSlammer.Set(0);
      leftSlammer.Set(0);
      Drive(0, 0, false);
    }
  }

  else if (m_autoSelected == kAutoShoot) {
    Shoot(startTime);
  }

  else if (m_autoSelected == kAutoTaxi){
    if (elapsedTime < 4)
      Drive(0.25, 0, false);
  } else {
    setpoint = INTAKE;
  }
}

// TELEOP ==================================================================
void Robot::TeleopInit() {

  //set all motors to 0
  shootMotor1.Set(0);
  shootMotor2.Set(0);
  shootMotor3.Set(0);
  shootMotor4.Set(0);
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
    setpoint = ZERO;
  } else if (opController.GetRawButtonPressed(4)){
    // amp
    setpoint = AKIMBO;
  } else if (opController.GetRawButtonPressed(2)){
    // akimbo 
    setpoint = SHOOT;
  } else if (opController.GetRawButtonPressed(3)){
    // intake
    setpoint = INTAKE;
  }

  // if the joystick is moved, disable pid
  if (abs(opController.GetRawAxis(1)) > 0.04){
    doPid = false;
  }else 
    doPid = true;

  RotateArm(opController.GetRawAxis(1));

  // if both joystick buttons are pressed, reset encoder
  if (opController.GetRawButton(9) && opController.GetRawButton(10)){
    m_armEncoder.SetPosition(0);
    setpoint = ZERO;
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
  frc::SmartDashboard::PutNumber("Arm angle", m_armEncoder.GetPosition());
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
  if (doPid){

    //calculate and set pid output
    double output = m_armController.Calculate(m_armEncoder.GetPosition(), setpoint);
    if (output > 1){
      output = 1;
    } else if (output < -1){
      output = -1;
    }
    rightSlammer.Set(output);
    leftSlammer.Set(output);
  } else {
    // set the joystick output, move the setpoint with the arm
    if (abs(speed) < 0.1){
      rightSlammer.Set(0);
      leftSlammer.Set(0);
      return;
    }
    rightSlammer.Set(speed);
    leftSlammer.Set(speed);
    setpoint = m_armEncoder.GetPosition();
  }
}

void Robot::Shoot(units::second_t startTime){
  double elapsedTime = frc::Timer::GetFPGATimestamp().value() - startTime.value();

  if (elapsedTime < 0.5){

    // rotate top wheel
    shootMotor1.Set(1.0);
    shootMotor2.Set(1.0);
  } else if (elapsedTime < 1.5){

    // rotate both wheels
    shootMotor1.Set(1.0);
    shootMotor2.Set(1.0);
    shootMotor3.Set(1.0);
    shootMotor4.Set(1.0);
  } else {

    // stop  
    isShooting = false;
  }

}

void Robot::RunShooter(double speed){
  shootMotor1.Set(speed * 0.15);
  shootMotor2.Set(speed * 0.15);
  shootMotor3.Set(speed * 0.15);
  shootMotor4.Set(speed * 0.15);
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif