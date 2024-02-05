#pragma once

#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/acceleration.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/Encoder.h>
#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/geometry/Pose2d.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/PIDController.h>
#include <AHRS.h>

using namespace units;

class RobotSubsystem{
    public:
        RobotSubsystem();
        void Update();

        void RunConveyor(double speed);

        void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
        void Drive(units::meters_per_second_t xSpeed,
                    units::radians_per_second_t rot);
        void JoystickDrive(double xSpeed, double zRotation, bool turnInPlace);
        void FollowTrajectory(frc::Trajectory trajectory);

        double GetLeftEncoder();
        double GetRightEncoder();
        frc::Pose2d GetPose();
        double GetTime();

        void Reset();
        void Reset(const frc::Pose2d& pose);

    private:
        const int lmotor_pwm_channel_1 = 9;
        const int lmotor_pwm_channel_2 = 8;
        const int rmotor_pwm_channel_1 = 0;
        const int rmotor_pwm_channel_2 = 1;
        const int conv_pwm_channel = 2;

        // electronics
        frc::PWMSparkMax m_leftMotor1{lmotor_pwm_channel_1};
        frc::PWMSparkMax m_rightMotor1{rmotor_pwm_channel_1};
        frc::PWMSparkMax m_leftMotor2{lmotor_pwm_channel_2};
        frc::PWMSparkMax m_rightMotor2{rmotor_pwm_channel_2};
        frc::PWMSparkMax m_conveyorMotor{conv_pwm_channel};

        frc::Encoder m_leftEncoder{1, 2};
        frc::Encoder m_rightEncoder{7, 8};
        AHRS m_gyro{frc::SPI::Port::kMXP};


        // drivetrain
        frc::DifferentialDrive m_robotDrive{m_leftMotor1, m_rightMotor1};
        frc::SlewRateLimiter<units::dimensionless::scalar> filter{2/1_s};


        // drivetrain controllers
        frc::DifferentialDriveKinematics m_kinematics{28_in};
        frc::DifferentialDriveOdometry m_odometry{
            m_gyro.GetRotation2d(), units::meter_t{m_leftEncoder.GetDistance()},
            units::meter_t{m_rightEncoder.GetDistance()}};

        frc::PIDController m_leftPIDController{1.0, 0.0, 0.0};
        frc::PIDController m_rightPIDController{1.0, 0.0, 0.0};
        frc::RamseteController m_controller{};
        frc::SimpleMotorFeedforward<units::meters> m_feedforward{1_V, 3_V / 1_mps};

        frc::Pose2d m_pose{};
        frc::Timer m_timer{};
};