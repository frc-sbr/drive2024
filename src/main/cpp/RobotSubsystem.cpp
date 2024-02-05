#include "RobotSubsystem.h"
#include "frc/Errors.h"
#include "frc/SPI.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

RobotSubsystem::RobotSubsystem() {
    m_leftMotor1.AddFollower(m_leftMotor2);
    m_rightMotor1.AddFollower(m_rightMotor2);

    m_rightMotor1.SetInverted(true);

    m_leftEncoder.Reset();
    m_rightEncoder.Reset();

    m_leftEncoder.SetDistancePerPulse(2 * std::numbers::pi * 0.0762 / 2048);
    m_rightEncoder.SetDistancePerPulse(2 * std::numbers::pi * 0.0762 / 2048);

    m_leftEncoder.SetReverseDirection(true);
    m_rightEncoder.SetReverseDirection(true);
}

void RobotSubsystem::Update() {
    // Get the rotation of the robot from the gyro.
    frc::Rotation2d gyroAngle = m_gyro.GetRotation2d();
    frc::SmartDashboard::PutNumber("Gyro Angle", gyroAngle.Degrees().value());

    // Update the pose
    m_pose = m_odometry.Update(gyroAngle,
        units::meter_t{m_leftEncoder.GetDistance()},
        units::meter_t{m_rightEncoder.GetDistance()});
}


void RobotSubsystem::RunConveyor(double speed){
    m_conveyorMotor.Set(speed * 0.3);
}

void RobotSubsystem::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  const auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  const auto rightFeedforward = m_feedforward.Calculate(speeds.right);
  const double leftOutput = m_leftPIDController.Calculate(
      m_leftEncoder.GetRate(), speeds.left.value());
  const double rightOutput = m_rightPIDController.Calculate(
      m_rightEncoder.GetRate(), speeds.right.value());

  m_leftMotor1.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightMotor1.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
}

void RobotSubsystem::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot) {
  SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void RobotSubsystem::JoystickDrive(double xSpeed, double zRotation, bool turnInPlace){
    if (xSpeed > 0){
        if (turnInPlace)
            m_robotDrive.CurvatureDrive(-filter.Calculate(xSpeed), zRotation, turnInPlace);
        else 
            m_robotDrive.CurvatureDrive(-filter.Calculate(xSpeed), -zRotation, turnInPlace);
    } else {
        m_robotDrive.CurvatureDrive(-filter.Calculate(xSpeed), zRotation, turnInPlace);
    }
}

void RobotSubsystem::FollowTrajectory(frc::Trajectory trajectory) {
    if (m_timer.Get() < trajectory.TotalTime()) {
        frc::Trajectory::State goal = trajectory.Sample(m_timer.Get());
        frc::ChassisSpeeds adjustedSpeeds = m_controller.Calculate(m_pose, goal);

        frc::SmartDashboard::PutNumber("chassis speed vx", adjustedSpeeds.vx.value());
        frc::SmartDashboard::PutNumber("chassis speed omega", adjustedSpeeds.omega.value());
        Drive(adjustedSpeeds.vx, adjustedSpeeds.omega);
    } else {
        Drive(0_mps, 0_rad_per_s);
    }
}

double RobotSubsystem::GetLeftEncoder(){
    return m_leftEncoder.GetDistance();
}

double RobotSubsystem::GetRightEncoder(){
    return m_rightEncoder.GetDistance();
}

frc::Pose2d RobotSubsystem::GetPose() {
    return m_pose;
}

double RobotSubsystem::GetTime() {
    return m_timer.Get().value();
}


void RobotSubsystem::Reset(){
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();

    m_timer.Reset();
    m_timer.Start();
}

void RobotSubsystem::Reset(const frc::Pose2d& pose) {
    m_odometry.ResetPosition(m_gyro.GetRotation2d(),
                           units::meter_t{m_leftEncoder.GetDistance()},
                           units::meter_t{m_rightEncoder.GetDistance()}, pose);
}