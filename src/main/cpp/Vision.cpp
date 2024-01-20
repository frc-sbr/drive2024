#include "Vision.hpp"
#include <units/length.h>

int initVision(){
	cap.open(0);
	if(!cap.isOpened()){
		// TODO add a driver station console log
		return -1;
	}

	detector.AddFamily("tag36h11");

	config.tagSize = tagPhysicalSize;
	config.fx = camFocalLength;
	config.fy = camFocalLength;
	config.cx = camCX;
	config.cy = camCY;

	poseEstimator.SetConfig(config);

	return 0;
}

int checkFrame(frc::DifferentialDrive* m_robotDrive){
	cap >> video;
	cv::cvtColor(video, gray, cv::COLOR_BGR2GRAY);

	frc::AprilTagDetector::Results res = frc::AprilTagDetect(detector, gray);

	for(const frc::AprilTagDetection *aprDetect : res){
		// mess with hamming, see if 1 or 2 works better
		// NOTE 2 might cause a lot of false positive
		// also maybe mess with using the decision margin to check for good results
		if(aprDetect->GetHamming() == 0){
			frc::Transform3d pose = poseEstimator.Estimate(*aprDetect);
			switch(aprDetect->GetId()){
				case 5:{
					
				}
				default: break;
			}
		}
	}

 	return 0;
}