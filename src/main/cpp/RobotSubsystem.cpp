#include "RobotSubsystem.h"

RobotSubsystem::RobotSubsystem(){
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

void RobotSubsystem::RunConveyor(double speed){
    m_conveyorMotor.Set(speed * 0.3);
}

double RobotSubsystem::GetLeftEncoder(){
    return m_leftEncoder.GetDistance();
}

double RobotSubsystem::GetRightEncoder(){
    return m_rightEncoder.GetDistance();
}

void RobotSubsystem::ResetEncoders(){
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
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

frc::Encoder& RobotSubsystem::GetClimbEncoder()
{
    return RobotSubsystem::climb_Encoder;
}