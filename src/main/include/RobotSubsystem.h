#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/acceleration.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/Encoder.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace units;

class RobotSubsystem{
    public:
        RobotSubsystem();
        void RunConveyor(double speed);
        void JoystickDrive(double xSpeed, double zRotation, bool turnInPlace);
        void Drive();
        double GetLeftEncoder();
        double GetRightEncoder();
        void ResetEncoders();
        frc::Encoder& GetClimbEncoder();

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

        frc::DifferentialDrive m_robotDrive{m_leftMotor1, m_rightMotor1};
        frc::DifferentialDriveKinematics m_kinematics{0.381_m};
        
        const int climber_pin_1 = 2;
        const int climber_pin_2 = 3;
        frc::Encoder climb_Encoder{climber_pin_1, climber_pin_2};
};