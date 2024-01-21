// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <thread>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/PS4Controller.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/filter/SlewRateLimiter.h>
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/acceleration.h>
#include <frc/motorcontrol/Spark.h>
#include "Vision.hpp"

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

 private:

  const int lmotor_pwm_channel_1 = 1;
  const int lmotor_pwm_channel_2 = 3;
  const int rmotor_pwm_channel_1 = 2;
  const int rmotor_pwm_channel_2 = 4;

  frc::Spark m_leftMotor1{lmotor_pwm_channel_1};
	frc::Spark m_rightMotor1{rmotor_pwm_channel_1};
  frc::Spark m_leftMotor2{lmotor_pwm_channel_2};
	frc::Spark m_rightMotor2{rmotor_pwm_channel_2};
	
	frc::DifferentialDrive m_robotDrive{m_leftMotor1, m_rightMotor1};
  frc::SlewRateLimiter<units::scalar> filter{6/1_s};

	frc::PS4Controller controller{0};
};
