#include "Vision.hpp"
#include <units/length.h>

int VisionThread(){
	cs::UsbCamera cam = frc::CameraServer::StartAutomaticCapture();
	
	cam.SetResolution(640, 480);

	cs::CvSink cvSink = frc::CameraServer::GetVideo();
	cs::CvSource outputStream = frc::CameraServer::PutVideo("Vision Cam", 640, 480);
	
	frc::AprilTagDetector detector;

	detector.AddFamily("tag36h11", 0);

	frc::AprilTagPoseEstimator::Config config;

	config.tagSize = tagPhysicalSize;
	config.fx = camFocalLength;
	config.fy = camFocalLength;
	config.cx = camCX;
	config.cy = camCY;

	frc::AprilTagPoseEstimator poseEstimator(config);

	cv::Mat video;
	cv::Mat gray;

	while(true){
		if(cvSink.GrabFrame(video) == 0){
			outputStream.NotifyError(cvSink.GetError());
			continue;
		}
		cv::cvtColor(video, gray, cv::COLOR_BGR2GRAY);

		cv::Size size = gray.size();

		frc::AprilTagDetector::Results detections = detector.Detect(size.width, size.height, size.data);

		for(const frc::AprilTagDetection* aprDetect : detections){
			// TODO mess with hamming, see if 1 or 2 works better
			// ! 2 might cause a lot of false positive
			// * also try using the decision margin to check for good results
			if(aprDetect->GetHamming() == 0){
				// visuals
				for(int i = 0; i < 4; i++){
					int j = (i+1) & 4;

					frc::AprilTagDetection::Point p1 = aprDetect->GetCorner(i);
					frc::AprilTagDetection::Point p2 = aprDetect->GetCorner(j);

					line(video, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), outlineColor, 2);
				}
				
				frc::Transform3d pose = poseEstimator.Estimate(*aprDetect);

				std::stringstream dashboardText;
				dashboardText << "Translate: " << units::length::to_string(pose.X())
											<< ", " << units::length::to_string(pose.Y())
											<< ", " << units::length::to_string(pose.Z())
											<< "Rotation: " << units::angle::to_string(pose.Rotation().X())
											<< ", " << units::angle::to_string(pose.Rotation().Y())
											<< ", " << units::angle::to_string(pose.Rotation().Z());

				frc::SmartDashboard::PutString(
					"pose_" + std::to_string(aprDetect->GetId()),
					dashboardText.str());

				if(frc::GetAlliance() == frc::Alliance::kRed){
					switch(aprDetect->GetId()){
						// tag 5, at the red amp
						case 5:{
						}
						default: break;
					}
				}
				else{
					switch(aprDetect->GetId()){
						// tag 6, at the blue amp
						case 6:{

						}
						default: break;
					}
				}
			}
		}
		outputStream.PutFrame(video);
	}

	return 0;
}