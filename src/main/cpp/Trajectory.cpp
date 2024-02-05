#include "Trajectory.h"
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>

frc::Trajectory Trajectory::m_autonTrajectory;

void Trajectory::GenerateAutonTrajectory() {
    const frc::Pose2d start{0_m, 0_m, 0_deg};
    const frc::Pose2d end{1_m, 0_m, 0_deg};

    frc::TrajectoryConfig config{1_mps, 1_mps_sq};
    m_autonTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        start, {}, end, config);
}