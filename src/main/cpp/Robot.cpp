// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <cameraserver/CameraServer.h>
#include <frc/TimedRobot.h>

#define TURBO_ENABLED 0

// ROBOT ===================================================================
void Robot::RobotInit() {
  frc::CameraServer::StartAutomaticCapture();

  m_autonomousCommand = PathPlannerAuto("New Auto").ToPtr();
}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("Left Encoder Speed", m_robotSubsystem.GetLeftEncoder());
  frc::SmartDashboard::PutNumber("Right Encoder Speed", m_robotSubsystem.GetRightEncoder());

  frc::SmartDashboard::PutNumber("Pose X", m_robotSubsystem.GetPose().X().value());
  frc::SmartDashboard::PutNumber("Pose Y", m_robotSubsystem.GetPose().Y().value());
  frc::SmartDashboard::PutNumber("Pose Rotation", m_robotSubsystem.GetPose().Rotation().Degrees().value());

  m_robotSubsystem.UpdateOdometry();
}

// AUTON  ==================================================================
void Robot::AutonomousInit() {
  m_robotSubsystem.ResetOdometry(m_robotSubsystem.GetPose());


}

void Robot::AutonomousPeriodic() {
  m_autonomousCommand.get()->Execute();
}

// TELEOP ==================================================================
void Robot::TeleopInit() {
  m_robotSubsystem.ResetOdometry(m_robotSubsystem.GetPose());
}

void Robot::TeleopPeriodic() {
  // #if TURBO_ENABLED
  //   m_robotSubsystem.JoystickDrive(
  //     driveController.GetRawAxis(3) - driveController.GetRawAxis(2), 
  //     driveController.GetRawAxis(0), 
  //     driveController.GetRawButton(1));
  // #else 
  //   m_robotSubsystem.JoystickDrive(
  //     (driveController.GetRawAxis(3) - driveController.GetRawAxis(2))/2, 
  //     driveController.GetRawAxis(0), 
  //     driveController.GetRawButton(1));
  // #endif

  m_robotSubsystem.Drive(
    meters_per_second_t{-driveController.GetRawAxis(1) * 1.0}, 
    radians_per_second_t{-driveController.GetRawAxis(4) * 3.0});

  m_robotSubsystem.RunConveyor(opController.GetRawAxis(3) - opController.GetRawAxis(2));
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif