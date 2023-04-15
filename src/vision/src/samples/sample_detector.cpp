#include "class_timer.hpp"
#include "class_detector.h"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <memory>
#include <thread>
#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Bool.h"




int main(int argc, char **argv)
{

	ros::init(argc, argv, "detection_publisher");
	ros::NodeHandle nh;

	    // Create a publisher for detections
	ros::Publisher detection_pub = nh.advertise<geometry_msgs::PoseArray>("detections", 1000);
	
	//create a publisher for navigation
	ros::Publisher nav_control_pub = nh.advertise<std_msgs::Bool>("vision_navigation_control", 1000);
	std_msgs::Bool nav_control_msg;
	
	ros::Publisher vision_ready_pub = nh.advertise<std_msgs::Bool>("nn1", 10);
	std_msgs::Bool vision_ready_msg;
	
	std::cout << "test" << std::endl;
	vision_ready_msg.data = false;
	vision_ready_pub.publish(vision_ready_msg);

	// Set the loop rate
	ros::Rate loop_rate(30); // 30 Hz

	Config config_v3;
	config_v3.net_type = YOLOV3;
	config_v3.file_model_cfg = "../configs/yolov3.cfg";
	config_v3.file_model_weights = "../configs/yolov3.weights";
	config_v3.calibration_image_list_file_txt = "../configs/calibration_images.txt";
	config_v3.inference_precison =FP32;
	config_v3.detect_thresh = 0.5;


	Config config_v4;
	config_v4.net_type = YOLOV4;
	config_v4.file_model_cfg = "../configs/yolov4.cfg";
	config_v4.file_model_weights = "../configs/yolov4.weights";
	config_v4.calibration_image_list_file_txt = "../configs/calibration_images.txt";
	config_v4.inference_precison = FP32;
	config_v4.detect_thresh = 0.5;

	Config config_v4_tiny;
	config_v4_tiny.net_type = YOLOV4_TINY;
	config_v4_tiny.detect_thresh = 0.5;
	config_v4_tiny.file_model_cfg = "../configs/yolov4-tiny-synth-mk1.cfg";
	config_v4_tiny.file_model_weights = "../configs/yolov4-tiny-synth-mk1.weights";
	config_v4_tiny.calibration_image_list_file_txt = "../configs/calibration_images.txt";
	config_v4_tiny.inference_precison = FP16;

	Config config_v5;
	config_v5.net_type = YOLOV5;
	config_v5.detect_thresh = 0.5;
	config_v5.file_model_cfg = "../configs/yolov5-5.0/yolov5s6.cfg";
	config_v5.file_model_weights = "../configs/yolov5-5.0/yolov5s6.weights";
	config_v5.calibration_image_list_file_txt = "../configs/calibration_images.txt";
	config_v5.inference_precison = FP32;

	std::unique_ptr<Detector> detector(new Detector());
	detector->init(config_v4_tiny);

	//vision_ready_msg.data = true;
	//vision_ready_pub.publish(vision_ready_msg);

    	cv::VideoCapture cap(0);
    	cv::Mat frame;


	//cv::Mat image0 = cv::imread("../configs/cardinal2.jpg", cv::IMREAD_UNCHANGED);
	//cv::Mat image1 = cv::imread("../configs/cardinal2.jpg", cv::IMREAD_UNCHANGED);
	std::vector<BatchResult> batch_res;
	Timer timer;
	while(cap.read(frame))
	{
		//prepare batch data
		std::vector<cv::Mat> batch_img;
		batch_img.push_back(frame);
		//batch_img.push_back(temp1);

		//detect
		timer.reset();
		detector->detect(batch_img, batch_res);
		timer.out("detect");
		
		geometry_msgs::PoseArray pose_array_msg;
		pose_array_msg.header.stamp = ros::Time::now();
		pose_array_msg.poses.clear();

		//disp
		for (int i=0;i<batch_img.size();++i)
		{
			for (const auto &r : batch_res[i])
			{
				std::cout <<"batch "<<i<< " id:" << r.id << " prob:" << r.prob << " rect:" << r.rect << std::endl;
				cv::rectangle(batch_img[i], r.rect, cv::Scalar(255, 0, 0), 2);
				std::stringstream stream;
				stream << std::fixed << std::setprecision(2) << "id:" << r.id << "  score:" << r.prob;
				cv::putText(batch_img[i], stream.str(), cv::Point(r.rect.x, r.rect.y - 5), 0, 0.5, cv::Scalar(0, 0, 255), 2);
				
				
								// Create and populate the Pose message
				geometry_msgs::Pose pose_msg;
				pose_msg.position.x = r.rect.x;
				pose_msg.position.y = r.rect.y;
				pose_msg.position.z = r.id;
				pose_msg.orientation.x = 0;
				pose_msg.orientation.y = 0;
				pose_msg.orientation.z = 0;
				pose_msg.orientation.w = 0;
				
				pose_array_msg.poses.push_back(pose_msg);
			}
			
					
			vision_ready_msg.data = true;
			vision_ready_pub.publish(vision_ready_msg);
			cv::namedWindow("image" + std::to_string(i), cv::WINDOW_NORMAL);
			cv::imshow("image"+std::to_string(i), batch_img[i]);
		}
		
		
		
		if (!pose_array_msg.poses.empty()) {
			nav_control_msg.data = true;
		    } 
		else 
		    {
			nav_control_msg.data = false;
		    }
		    
		nav_control_pub.publish(nav_control_msg);
		
		detection_pub.publish(pose_array_msg);
		cv::waitKey(10);
		
		ros::spinOnce();
        	loop_rate.sleep();
	}
}
