#pragma once

#include <opencv2/opencv.hpp>

#include <frc/apriltag/AprilTagDetector_cv.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>

#include <frc/geometry/Transform3d.h>

#include <frc/drive/DifferentialDrive.h>

// the tags irl size in inches
const units::inch_t tagPhysicalSize = 6_in;

// TODO update focalLength to be accurate to cam (mm)
const double camFocalLength = 3.67;
// TODO update values to be accurate to cam
const double camCX = 1920*0.5;
const double camCY = 1080*0.5;

inline cv::Mat video;
inline cv::Mat gray;
inline cv::VideoCapture cap;

inline frc::AprilTagDetector detector;
inline frc::AprilTagPoseEstimator::Config config;
inline frc::AprilTagPoseEstimator poseEstimator(config);

int initVision();

int checkFrame(frc::DifferentialDrive* m_robotDrive);