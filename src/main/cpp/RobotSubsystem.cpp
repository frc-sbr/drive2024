#include "RobotSubsystem.h"
#include "frc/Errors.h"
#include "frc/SPI.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>

using namespace pathplanner;

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

    m_gyro.Reset();

    AutoBuilder::configureRamsete(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetSpeed(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ Drive(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        ReplanningConfig(), // Default path replanning config. See the API for the options here
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}

void RobotSubsystem::RunConveyor(double speed){
    m_conveyorMotor.Set(speed * 0.3);
}

void RobotSubsystem::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(),
                    units::meter_t{m_leftEncoder.GetDistance()},
                    units::meter_t{m_rightEncoder.GetDistance()});
}

void RobotSubsystem::StopMotors(){
    m_leftMotor1.Set(0);
    m_rightMotor1.Set(0);   
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

void RobotSubsystem::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot){
    frc::DifferentialDriveWheelSpeeds speeds = m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot});

    const auto leftFeedforward = m_feedforward.Calculate(speeds.left);
    const auto rightFeedforward = m_feedforward.Calculate(speeds.right);
    const double leftOutput = m_leftPIDController.Calculate(
        m_leftEncoder.GetRate(), speeds.left.value());
    const double rightOutput = m_rightPIDController.Calculate(
        m_rightEncoder.GetRate(), speeds.right.value());
    
    m_leftMotor1.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
    m_rightMotor1.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
}

void RobotSubsystem::Drive(frc::ChassisSpeeds chassisSpeed){
    frc::DifferentialDriveWheelSpeeds speeds = m_kinematics.ToWheelSpeeds(chassisSpeed);    

    const auto leftFeedforward = m_feedforward.Calculate(speeds.left);
    const auto rightFeedforward = m_feedforward.Calculate(speeds.right);
    const double leftOutput = m_leftPIDController.Calculate(
        m_leftEncoder.GetRate(), speeds.left.value());
    const double rightOutput = m_rightPIDController.Calculate(
        m_rightEncoder.GetRate(), speeds.right.value());
    
    m_leftMotor1.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
    m_rightMotor1.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
}

frc::ChassisSpeeds RobotSubsystem::GetSpeed(){
    return m_kinematics.ToChassisSpeeds(
        {units::meters_per_second_t{m_leftEncoder.GetRate()},
        units::meters_per_second_t{m_rightEncoder.GetRate()}}
    );
}

double RobotSubsystem::GetLeftEncoder(){
    return m_leftEncoder.GetRate();
}

double RobotSubsystem::GetRightEncoder(){
    return m_rightEncoder.GetRate();
}

void RobotSubsystem::ResetOdometry(frc::Pose2d pose){
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();

    m_odometry.ResetPosition(m_gyro.GetRotation2d(), units::meter_t{m_leftEncoder.GetDistance()}, units::meter_t{m_rightEncoder.GetDistance()}, pose);
}

frc::Pose2d RobotSubsystem::GetPose() {
  return m_odometry.GetPose();
}