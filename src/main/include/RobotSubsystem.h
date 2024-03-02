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
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/SubsystemBase.h>
#include <AHRS.h>

using namespace units;

class RobotSubsystem : public frc2::SubsystemBase{
    public:
        RobotSubsystem();

        void RunConveyor(double speed);
        void JoystickDrive(double xSpeed, double zRotation, bool turnInPlace);

        double GetLeftEncoder();
        double GetRightEncoder();
        void ResetOdometry(frc::Pose2d pose);
        void Drive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot);
        void Drive(frc::ChassisSpeeds chassisSpeed);
        frc::ChassisSpeeds GetSpeed();
        void StopMotors();
        void UpdateOdometry();
        void SetMotorVoltage(units::volt_t left, units::volt_t right);

        frc::Pose2d GetPose();

    private:
        const int lmotor_pwm_channel_1 = 9;
        const int lmotor_pwm_channel_2 = 8;
        const int rmotor_pwm_channel_1 = 0;
        const int rmotor_pwm_channel_2 = 1;
        const int conv_pwm_channel = 2;

        frc::PWMSparkMax m_leftMotor1{lmotor_pwm_channel_1};
        frc::PWMSparkMax m_rightMotor1{rmotor_pwm_channel_1};
        frc::PWMSparkMax m_leftMotor2{lmotor_pwm_channel_2};
        frc::PWMSparkMax m_rightMotor2{rmotor_pwm_channel_2};
        frc::PWMSparkMax m_conveyorMotor{conv_pwm_channel};
        frc::SlewRateLimiter<units::dimensionless::scalar> filter{1/1_s};

        frc::Encoder m_leftEncoder{5, 6};
        frc::Encoder m_rightEncoder{1, 2};

        AHRS m_gyro{frc::SPI::Port::kMXP};

        frc::DifferentialDrive m_robotDrive{m_leftMotor1, m_rightMotor1};
        frc::DifferentialDriveKinematics m_kinematics{28_in};
        frc::DifferentialDriveOdometry m_odometry{{}, units::meter_t{m_leftEncoder.GetDistance()}, units::meter_t{m_rightEncoder.GetDistance()}};
        frc::PIDController m_leftPIDController{1.0, 0.0, 0.0};
        frc::PIDController m_rightPIDController{1.0, 0.0, 0.0};
        frc::SimpleMotorFeedforward<units::meters> m_feedforward{0.69_V, 2.63_V / 1_mps};

        frc::RamseteController m_controller;
};