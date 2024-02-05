#include "RobotSubsystem.h"
#include "frc/Errors.h"
#include "frc/SPI.h"

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

    try {
        m_gyro = new AHRS(frc::SPI::Port::kMXP);
    } catch (std::exception ex ) {
        std::string err_string = "Error instantiating navX-MXP:  ";
        err_string += ex.what();
        FRC_ReportError(frc::err::Error, "{}", err_string);
    }
}

void RobotSubsystem::Update() {
    // Get the rotation of the robot from the gyro.
    frc::Rotation2d gyroAngle = m_gyro->GetRotation2d();
    frc::SmartDashboard::PutNumber("Gyro Angle", gyroAngle.Degrees().value());

    // Update the pose
    m_pose = m_odometry.Update(gyroAngle,
        units::meter_t{m_leftEncoder.GetDistance()},
        units::meter_t{m_rightEncoder.GetDistance()});
}


void RobotSubsystem::RunConveyor(double speed){
    m_conveyorMotor.Set(speed * 0.3);
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


double RobotSubsystem::GetLeftEncoder(){
    return m_leftEncoder.GetDistance();
}

double RobotSubsystem::GetRightEncoder(){
    return m_rightEncoder.GetDistance();
}

frc::Pose2d RobotSubsystem::GetPose() {
    return m_pose;
}


void RobotSubsystem::Reset(){
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();

    m_odometry.ResetPosition(m_gyro->GetRotation2d(), units::meter_t{m_leftEncoder.GetDistance()}, units::meter_t{m_rightEncoder.GetDistance()}, {});
}