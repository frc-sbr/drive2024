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
#include <AHRS.h>

using namespace units;

class RobotSubsystem{
    public:
        RobotSubsystem();
        void Update();

        void RunConveyor(double speed);
        void JoystickDrive(double xSpeed, double zRotation, bool turnInPlace);

        double GetLeftEncoder();
        double GetRightEncoder();
        frc::Pose2d GetPose();
        void Reset();

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
        frc::SlewRateLimiter<units::dimensionless::scalar> filter{2/1_s};

        frc::Encoder m_leftEncoder{1, 2};
        frc::Encoder m_rightEncoder{7, 8};
        AHRS *m_gyro;

        frc::DifferentialDrive m_robotDrive{m_leftMotor1, m_rightMotor1};
        // frc::DifferentialDriveKinematics m_kinematics{0.381_m};
        frc::Pose2d m_pose{};
        frc::DifferentialDriveOdometry m_odometry{{}, units::meter_t{m_leftEncoder.GetDistance()}, units::meter_t{m_rightEncoder.GetDistance()}};

        frc::RamseteController m_controller;
};