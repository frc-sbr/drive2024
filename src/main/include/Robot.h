// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
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
#include <frc2/command/CommandPtr.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/Encoder.h>
#include "rev/CANSparkMax.h"
#include "rev/SparkRelativeEncoder.h"
#include <frc/Timer.h>

using namespace pathplanner;
using namespace units;


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void Drive(double xSpeed, double zRotation, bool turnInPlace);
  void RotateArm(double speed);
  void RunShooter(double speed);
  void Shoot(units::second_t startTime);

 private:
  frc::PWMSparkMax m_leftMotor1{9};
  frc::PWMSparkMax  m_rightMotor1{0};
  frc::PWMSparkMax m_leftMotor2{8};
  frc::PWMSparkMax  m_rightMotor2{1};
  frc::SlewRateLimiter<units::dimensionless::scalar> filter{1/1_s};

  frc::DifferentialDrive m_robotDrive{m_leftMotor1, m_rightMotor1};

  rev::CANSparkMax rightSlammer{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax leftSlammer{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax shootMotor1{5, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax shootMotor2{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax shootMotor3{6, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax shootMotor4{7, rev::CANSparkMax::MotorType::kBrushed};

  rev::SparkRelativeEncoder m_armEncoder{rightSlammer.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)};

  // frc::ArmFeedforward m_armFeedforward{0.0_V, 0.0_V, 1_V / 1_rad_per_s, 0_V};
  frc::PIDController m_armController{2.5, 0.5, 0.05};

  frc::Joystick driveController{0};
  frc::Joystick opController{1};

  bool isShooting = false;
  bool doPid = true;
  units::second_t startTime = 0_s;
  double setpoint = 0;

  const std::string kAutoTaxi = "kAutoTaxi";
  const std::string kAutoShoot = "kAutoShoot";
  const std::string kAutoTaxiShoot = "kAutoTaxiShoot";
  const std::string kAutoWait = "kAutoWait";

  const double SHOOT = -0.586;
  const double AKIMBO = -1.582;
  const double INTAKE = -0.333; 
  const double ZERO = 0.0;
  std::string m_autoSelected;

  frc::SendableChooser<std::string> m_chooser;
};