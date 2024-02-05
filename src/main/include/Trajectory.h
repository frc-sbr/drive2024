#pragma once
#include <frc/trajectory/Trajectory.h>

class Trajectory {
public:
    static void GenerateAutonTrajectory();
    
    static frc::Trajectory m_autonTrajectory;
};