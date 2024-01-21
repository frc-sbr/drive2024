#pragma once

#include <cameraserver/CameraServer.h>
#include <opencv2/opencv.hpp>

#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>

#include <frc/geometry/Transform3d.h>

#include <frc/drive/DifferentialDrive.h>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

int VisionThread();

// actually important values

// the tags irl size in inches
const units::inch_t tagPhysicalSize = 6_in;

// TODO update focalLength to be accurate to cam (mm)
const double camFocalLength = 3.67;
// TODO update values to be accurate to cam
const double camCX = 640*0.5;
const double camCY = 480*0.5;

// less important visuals for driver to see 
cv::Scalar outlineColor = cv::Scalar(0, 255, 0);
cv::Scalar crossColor = cv::Scalar(0, 0, 255);