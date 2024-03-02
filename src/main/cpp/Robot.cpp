// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <cameraserver/CameraServer.h>
#include <frc/TimedRobot.h>

#define TURBO_ENABLED 1

// ROBOT ===================================================================
void Robot::RobotInit() {
  frc::CameraServer::StartAutomaticCapture();
}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("Left Encoder Speed", m_robotSubsystem.GetLeftEncoder());
  frc::SmartDashboard::PutNumber("Right Encoder Speed", m_robotSubsystem.GetRightEncoder());
  frc::SmartDashboard::PutNumber("Motor Voltage", speed.value());

  // frc::SmartDashboard::PutNumber("Pose X", m_robotSubsystem.GetPose().X().value());
  // frc::SmartDashboard::PutNumber("Pose Y", m_robotSubsystem.GetPose().Y().value());
  // frc::SmartDashboard::PutNumber("Pose Rotation", m_robotSubsystem.GetPose().Rotation().Degrees().value());

  m_robotSubsystem.UpdateOdometry();
}

// AUTON  ==================================================================
void Robot::AutonomousInit() {
  m_robotSubsystem.ResetOdometry(m_robotSubsystem.GetPose());


}

void Robot::AutonomousPeriodic() {
}

// TELEOP ==================================================================
void Robot::TeleopInit() {
  m_robotSubsystem.ResetOdometry(m_robotSubsystem.GetPose());
}

void Robot::TeleopPeriodic() {
  //// NORMAL TELEOP
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
  // m_robotSubsystem.RunConveyor(opController.GetRawAxis(3) - opController.GetRawAxis(2));

  ////FEEDFORWARD AND FEEDBACK TELEOP
  // m_robotSubsystem.Drive(
  //   meters_per_second_t{-driveController.GetRawAxis(1) * 1.0}, 
  //   radians_per_second_t{-driveController.GetRawAxis(4) * 3.0}); 

  ////VOLTAGE TELEOP
  // if (driveController.GetRawButtonPressed(1)){
  //   speed += 0.2_V;
  // }
  // if (driveController.GetRawButtonPressed(2)){
  //   speed = 0.1_V;
  // }

  // m_robotSubsystem.SetMotorVoltage(speed, speed);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif