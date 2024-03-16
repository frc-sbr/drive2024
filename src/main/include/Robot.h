// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/acceleration.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/AddressableLED.h>
#include "RobotSubsystem.h"

using namespace units;


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void RunConveyor();
  void Drive();

  void LEDRainbow();
  void Solid(int r, int g, int b);

 private:
  RobotSubsystem m_robotSubsystem;

	frc::Joystick driveController{1};
  frc::Joystick opController{2};

  frc::Encoder m_leftEncoder{1, 2};

  const int led_pwm_channel = 4;
  const double climber_limit = 69420.0;

  static constexpr int kLength = 60;
  frc::AddressableLED m_led{led_pwm_channel};
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;
  int firstPixelHue = 0;
};