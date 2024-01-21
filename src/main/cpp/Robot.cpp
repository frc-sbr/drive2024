// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

void Robot::RobotInit() {
  m_leftMotor1.AddFollower(m_leftMotor2);
  m_rightMotor1.AddFollower(m_rightMotor2);

  m_rightMotor1.SetInverted(true);

  //lighting
  m_led.SetLength(kLength);
  m_led.SetData(m_ledBuffer);
  m_led.Start();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  //lighting
  Rainbow();
  m_led.SetData(m_ledBuffer); //flushes buffer kinda? Do it after a LED func call
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  m_robotDrive.ArcadeDrive(-filter.Calculate(controller.GetLeftY()), -controller.GetLeftX());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

void Robot::Rainbow() 
{
  for (int i = 0; i < kLength; i++) {
    const auto pixelHue = (firstPixelHue + (i * 180 / kLength)) % 180;
    m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
  }
  firstPixelHue += 3;
  firstPixelHue %= 180;
}

void Robot::Solid(int r, int g, int b)
{
  for (int i = 0; i < kLength; i++)
  {
    m_ledBuffer[i].SetRGB(r, g, b);
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif