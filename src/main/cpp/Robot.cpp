// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <cameraserver/CameraServer.h>
#include <frc/TimedRobot.h>

#define TURBO_ENABLED 1

void Robot::RobotInit() {
  frc::CameraServer::StartAutomaticCapture();

  m_led.SetLength(kLength);
  m_led.SetData(m_ledBuffer);
  m_led.Start();
}

void Robot::RobotPeriodic() {
  double height = m_robotSubsystem.GetClimbEncoder().GetDistance();
  if (height < climber_limit)
  {
    Solid(0, 205, 120); // green
  }
  else
    Solid(255, 0 , 0); // not green

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

  m_robotSubsystem.RunConveyor(opController.GetRawAxis(3) - opController.GetRawAxis(2));
}

void Robot::Solid(int r, int g, int b)
{
  for (int i = 0; i < kLength; i++)
  {
    m_ledBuffer[i].SetRGB(r, g, b);
  }
}

void Robot::LEDRainbow(){
  for (int i = 0; i < kLength; i++) {
    const auto pixelHue = (firstPixelHue + (i * 180 / kLength)) % 180;
    m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
  }
  firstPixelHue += 3;
  firstPixelHue %= 180;

  m_led.SetData(m_ledBuffer);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif