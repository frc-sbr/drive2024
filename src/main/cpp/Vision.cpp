#include "Vision.hpp"
#include <units/length.h>

int VisionThread(frc::DifferentialDrive& m_robotDrive){
	cs::UsbCamera cam = frc::CameraServer::StartAutomaticCapture(0);

	cam.SetVideoMode(cs::VideoMode::kGray, 640, 480, 30);
	
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

		frc::AprilTagDetector::Results detections = detector.Detect(size.width, size.height, gray.data);

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

				// pose.Z() is camera's distance to the tag
				// pose.Y() is downwards relative to the camera
				// pose.X() is right relative to the camera
				// Unsure about rotation, pose.Rotation().Y() is possibly yaw 
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

				if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){
					switch(aprDetect->GetId()){
						// tag 5, at the red amp
						case 5:{
							if(aprDetect->GetCenter().x >= 320+alignmentBuffer){
								// we too far right, gotta rotate left
								m_robotDrive.ArcadeDrive(0.0, -0.1);
							}
							else if(aprDetect->GetCenter().x <= 320-alignmentBuffer){
								// we too far left, gotta rotate right
								m_robotDrive.ArcadeDrive(0.0, 0.1);
							}
							else{
								// in alignment buffer (aligned)
								frc::SmartDashboard::PutString("tag_" + std::to_string(aprDetect->GetId()), "aligned");
							}
						}
						default: break;
					}
				}
				else{
					switch(aprDetect->GetId()){
						// tag 6, at the blue amp
						case 6:{
							if(aprDetect->GetCenter().x >= 320+alignmentBuffer){
								// we too far right, gotta rotate left
								m_robotDrive.ArcadeDrive(0.0, -0.1);
							}
							else if(aprDetect->GetCenter().x <= 320-alignmentBuffer){
								// we too far left, gotta rotate right
								m_robotDrive.ArcadeDrive(0.0, 0.1);
							}
							else{
								// in alignment buffer (aligned)
								frc::SmartDashboard::PutString("tag_" + std::to_string(aprDetect->GetId()), "aligned");
							}
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