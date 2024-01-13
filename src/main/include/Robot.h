// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

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

  const int lmotor_can_id_1 = 1;
  const int lmotor_can_id_2 = 2;
  const int rmotor_can_id_1 = 3;
  const int rmotor_can_id_2 = 4;

  rev::CANSparkMax m_leftMotor1{lmotor_can_id_1, rev::CANSparkLowLevel::MotorType::kBrushless};
	rev::CANSparkMax m_rightMotor1{rmotor_can_id_1, rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotor2{lmotor_can_id_2, rev::CANSparkLowLevel::MotorType::kBrushless};
	rev::CANSparkMax m_rightMotor2{rmotor_can_id_2, rev::CANSparkLowLevel::MotorType::kBrushless};
	
	frc::DifferentialDrive m_robotDrive{m_leftMotor1, m_rightMotor1};
  frc::SlewRateLimiter<units::scalar> filter{3/1_s};

	frc::PS4Controller controller{0};
	units::second_t start_time;
	units::second_t end_time;
  std::string m_autoSelected;
};