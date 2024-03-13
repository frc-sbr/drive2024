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
#include <frc2/command/CommandPtr.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include "rev/CANSparkMax.h"

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
  void RunShooter(double speed, bool shoot);

 private:
  frc::PWMSparkMax m_leftMotor1{9};
  frc::PWMSparkMax  m_rightMotor1{0};
  frc::PWMSparkMax m_leftMotor2{8};
  frc::PWMSparkMax  m_rightMotor2{1};
  frc::SlewRateLimiter<units::dimensionless::scalar> filter{1/1_s};

  frc::DifferentialDrive m_robotDrive{m_leftMotor1, m_rightMotor1};

  frc::PWMSparkMax rightSlammer{2};
  frc::PWMSparkMax leftSlammer{3};
  rev::CANSparkMax shootMotor1{6, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax shootMotor2{4, rev::CANSparkMax::MotorType::kBrushless};

  frc::Joystick driveController{1};
  frc::Joystick opController{2};
};