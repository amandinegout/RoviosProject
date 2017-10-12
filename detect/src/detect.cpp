#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <sensor_msgs/image_encodings.h>
#include <array>
#include "pantiltzoom.hpp"
#include <cmath>
#include <detect/PanTilts.h>
#include <detect/PanTilt.h>
#include <axis_camera/Axis.h>

#define NB_COLORS 4

class Detector {
    protected:
        ros::NodeHandle nh_;
        ros::Publisher pantilt_pub;
        ros::Subscriber pantilt_sub;
        image_transport::Publisher pub1;
        image_transport::Publisher pub2;
        image_transport::Publisher pub3;
        image_transport::Publisher pub4;
        image_transport::Subscriber sub;
        std::string image_topic;
        int tolerance;
        double iris_param;
        int H_min;
        int S_min;
        int V_min;
        int H_max;
        int S_max;
        int V_max;
        cv::Mat image_mat;
        cv_bridge::CvImageConstPtr image_det;
        int mat_size_x;
		int mat_size_y;
		std::array<cv::Scalar,NB_COLORS> colors;
		cv::Mat hsv;
		cv::Mat detection; 
		cv::Mat cleaned; 
		cv::Mat frame;
		cv::Scalar hsv_min;
		cv::Scalar hsv_max; 
		cv::Mat open_elem;
		cv::Mat close_elem;
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		std::vector<cv::Point> centers;
		int sum_x;
		int sum_y;
		detect::PanTilts pantilts;
		double zoom;
		double pan;
		double tilt;
		std::string camera_state;

    public:
		
        Detector() : nh_("~") {
			nh_.param("iris_param",iris_param,336.0);
			nh_.param("image_topic",image_topic,std::string("/image_raw"));
			nh_.param("H_min",H_min,150);
			nh_.param("S_min",S_min,130);
			nh_.param("V_min",V_min,240);
			nh_.param("H_max",H_max,200);
			nh_.param("S_max",S_max,160);
			nh_.param("V_max",V_max,255);
			nh_.param("tolerance",tolerance,10);
			nh_.param("zoom",zoom,2988.0);
			nh_.param("camera_state",camera_state,std::string("/state"));
			
			image_transport::ImageTransport it(nh_);
			sub = it.subscribe(image_topic, 1, &Detector::imageCallback,this);
			colors = {{cv::Scalar(255,0,0),
				{0,255,0},
				{0,0,255},
				{255,255,0}}};
				
			// Pink color range
			hsv_min = cv::Scalar(H_min, S_min,  V_min);
			hsv_max = cv::Scalar(H_max, S_max, V_max);
			
			// Morpho elements
			open_elem  = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
			close_elem = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(9,9));
			
			// Publishers and subscribers
            pub1 = it.advertise("detection", 1);
            pub2 = it.advertise("clean_up", 1);
            pub3 = it.advertise("contours", 1);
            pub4 = it.advertise("imagehsv", 1);
            pantilt_sub = nh_.subscribe(camera_state,1,&Detector::callback_pantilt,this);
            pantilt_pub = nh_.advertise<detect::PanTilts>("PanTilts",1);
			
        }
        
        // pantiltzoom outputs the pan/tilt position corresponding to a given position of the image
        std::pair<double,double> pantiltzoom(double u, double v, double u0,  double v0,
				     double pan, double tilt, double zoom){
		  double focale,theta,alpha0,beta0,alpha,beta;
		  double x,y,z,X,Y,Z,norme;
		  theta=4.189301e+001-6.436043e-003*zoom+2.404497e-007*zoom*zoom;
		  focale=u0/std::tan((M_PI*theta/180.0)/2);

		  x=u-u0;y=v-v0;z=focale;
		  norme=std::sqrt(x*x+y*y+z*z);
		  x/=norme;y/=norme;z/=norme;

		  beta0=-(M_PI*pan/180.0);
		  alpha0=-(M_PI*tilt/180.0);
		  X=std::cos(beta0)*x+std::sin(alpha0)*std::sin(beta0)*y-std::cos(alpha0)*std::sin(beta0)*z;
		  Y=std::cos(alpha0)*y+std::sin(alpha0)*z;
		  Z=std::sin(beta0)*x-std::sin(alpha0)*std::cos(beta0)*y+std::cos(alpha0)*std::cos(beta0)*z;
		  alpha=std::atan2(Y,sqrt(X*X+Z*Z));
		  beta=-std::atan2(X,Z);

		  return {-(180.0*beta/M_PI), -(180.0*alpha/M_PI)};
		}
		
		// This function is called everytime the camera publishes its state
		// we then store the updated state
		void callback_pantilt(const axis_camera::Axis& msg){
			pan  = msg.pan;
			tilt = msg.tilt;
		}
		
		// Function called when the camera publishes an image
        void imageCallback(const sensor_msgs::ImageConstPtr& msg){
		  try {
			image_det = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
			image_mat = image_det->image;
			mat_size_x = image_mat.rows;
			mat_size_y = image_mat.cols;
			// Convert image to hsv and publish it
			cv::cvtColor(image_mat, hsv, CV_BGR2HSV);   
			sensor_msgs::ImagePtr imagehsv = cv_bridge::CvImage(std_msgs::Header(), "mono8", hsv).toImageMsg();
			pub4.publish(imagehsv);
			
			// Display HSV value of the middle of hsv image (useful to calibrate the range for the chosen color)
			//ROS_INFO("The HSV is : H: %f , S: %f, V: %f",(double)vect2[0], (double)vect2[1], (double)vect2[2]);
			
			// Detection: select in-range pixels and publish it
			cv::inRange(hsv,hsv_min,hsv_max,detection); 
			sensor_msgs::ImagePtr img_detection = cv_bridge::CvImage(std_msgs::Header(), "mono8", detection).toImageMsg();
			pub1.publish(img_detection);
			// Clean-up of detection image with opening-closing morphologic operation and publication
			cv::erode (detection,detection,open_elem);  // opening
			cv::dilate(detection,detection,open_elem);  
			cv::dilate(detection,detection,close_elem); // closing
			cv::erode (detection,detection,close_elem); 
			sensor_msgs::ImagePtr img_detection_clean = cv_bridge::CvImage(std_msgs::Header(), "mono8", detection).toImageMsg();
			pub2.publish(img_detection_clean);
			// Contour selection
			cv::findContours(detection, 
				contours, hierarchy, 
				CV_RETR_TREE, 
				CV_CHAIN_APPROX_SIMPLE, 
				cv::Point(0, 0));
			cv::Mat display = cv::Mat::zeros(detection.size(), CV_8UC3 );
			centers.clear();
			// Computation of the center of contour and Pan tilt definition
			std::vector<detect::PanTilt> pantilt_vect;
			for(unsigned int i = 0; i< contours.size(); i++){
				sum_x = 0;
				sum_y = 0;
				for(unsigned int j = 0; j< contours[i].size(); j++){
					sum_x += contours[i][j].x;
					sum_y += contours[i][j].y;
				}
				cv::Point center = cv::Point(sum_x/contours[i].size(),sum_y/contours[i].size());
				centers.push_back(center);
				//ROS_INFO("x= %f , y= %f, center_x= %f, center_y= %f",(double)center.x, (double)center.y, (double)mat_size_x/2 ,(double)mat_size_y/2);
				// Let's use pantiltzoom to convert the positions of the centers in the image into pantilt coordinates
				std::pair<double,double> pantiltzoom = Detector::pantiltzoom((double)center.x,(double)center.y,(double)mat_size_y/2,(double)mat_size_x/2,pan,tilt,zoom);
				detect::PanTilt pantilt;
				pantilt.pan = pantiltzoom.first;
				pantilt.tilt = pantiltzoom.second;
				pantilt_vect.push_back(pantilt);
			}
			pantilts.pantilts = pantilt_vect;
			contours.push_back(centers);
			// Display of the found centers
			for(unsigned int i = 0; i< contours.size(); i++)
				cv::drawContours(display, contours,i, colors[i%NB_COLORS],3);
			sensor_msgs::ImagePtr img_display = cv_bridge::CvImage(std_msgs::Header(), "bgr8", display).toImageMsg();
			pub3.publish(img_display);
			// Publication of the pantilts positions of the detected robots
			pantilt_pub.publish(pantilts);	
		  } catch (cv_bridge::Exception& e){
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		  }
		}
        			

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"detector");
    Detector det;
    
	ros::spin();

    return 0;
}
