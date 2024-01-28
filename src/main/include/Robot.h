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
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/acceleration.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/AddressableLED.h>

using namespace units;


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
  void RunConveyor(bool forward);

 private:

  const int lmotor_pwm_channel_1 = 9;
  const int lmotor_pwm_channel_2 = 8;
  const int rmotor_pwm_channel_1 = 0;
  const int rmotor_pwm_channel_2 = 1;
  const int conv_pwm_channel = 2;

  frc::Spark m_leftMotor1{lmotor_pwm_channel_1};
	frc::Spark m_rightMotor1{rmotor_pwm_channel_1};
  frc::Spark m_leftMotor2{lmotor_pwm_channel_2};
	frc::Spark m_rightMotor2{rmotor_pwm_channel_2};
  frc::Spark m_conveyorMotor{conv_pwm_channel};

	frc::DifferentialDrive m_robotDrive{m_leftMotor1, m_rightMotor1};
  frc::SlewRateLimiter<units::scalar> filter{6/1_s};

	frc::Joystick controller{1};
};