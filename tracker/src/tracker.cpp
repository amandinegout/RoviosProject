#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <array>
#include <cmath>
#include <detect/PanTilts.h>
#include <detect/PanTilt.h>
#include <axis_camera/Axis.h>
#include <std_srvs/Empty.h>
#include "tracker/Mode.h"

class Tracker {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber pantilt_sub;
        ros::Subscriber state_sub;
        ros::Publisher command_pub;
		detect::PanTilts pantilts;
		double pan_state;
		double tilt_state;
		std::string pantilt;
		std::string mode;
		std::string state;
		std::string command_camera;
		bool up_to_date;
		std::string default_mode;
		

    public:
		
        Tracker() : nh_("~") {
			nh_.param("pantilt",pantilt,std::string("/detect/PanTilts"));
			nh_.param("command_camera",command_camera,std::string("/cmd"));
			nh_.param("state",state,std::string("/state"));
			nh_.param("default_mode",default_mode,std::string(""));
			mode = default_mode;
            pantilt_sub = nh_.subscribe(pantilt,1,&Tracker::callback_pantilt,this);
            state_sub = nh_.subscribe(state,1,&Tracker::callback_state,this);
            command_pub = nh_.advertise<axis_camera::Axis>(command_camera,1);
            up_to_date = false;
        }
        
        // Callback updating the camera state (pan, tilt)
        void callback_state(const axis_camera::Axis& msg){
			pan_state  = msg.pan;
			tilt_state = msg.tilt;
			up_to_date=true;
		}
        
		// Callback for all three possible modes : can/search/track sending pan/tilt msg to camera iff the state of the camera was upated
        void callback_pantilt(const detect::PanTilts& msg){
			if (up_to_date){
				axis_camera::Axis command_msg;
				std::vector<detect::PanTilt> pantilt_vect = msg.pantilts;
			
				if (mode == std::string("track")){
					ROS_INFO("pan = %f, tilt = %f",(double)pan_state,(double)tilt_state);
					if (pantilt_vect.size() == 0){
						ROS_INFO("No robot is detected. Not moving.");
					} else {
						// At least one robot detected
						int index = 0;
						double move_min = std::pow(pantilt_vect[0].pan - pan_state,2) + std::pow(pantilt_vect[0].tilt - tilt_state,2);
						// For each detected robot, computation of the necessary move to center the caera on it
						for (unsigned int i = 0; i< pantilt_vect.size();i++){
							double current_move = std::pow(pantilt_vect[i].pan - pan_state,2) + std::pow(pantilt_vect[i].tilt - tilt_state,2);
							ROS_INFO("Move for robot number %d is %f",(unsigned int) i, (double) current_move);
							// Selection of the minimum move to perform
							if (move_min>current_move){
								move_min = current_move;
								index = i;
							}
						}
						ROS_INFO("Robot number %d chosen with move min = %f",(unsigned int) index, (double) move_min);
						ROS_INFO("Robot detected. Moving to pan = %f and tilt = %f", (double)pantilt_vect[index].pan,(double)pantilt_vect[index].tilt);
						command_msg.pan = pantilt_vect[index].pan;
						command_msg.tilt = pantilt_vect[index].tilt;
						command_pub.publish(command_msg);
						ros::Duration(2.5).sleep();
						up_to_date = false;
					}
				}
				else if (mode == std::string("search")) {
					ROS_INFO("pan = %f, tilt = %f",(double)pan_state,(double)tilt_state);
					// Scan of the area as long as no robot is detected
					if (pantilt_vect.size() == 0){
						if (pan_state>90.0){
							command_msg.pan = -90.0;
							if (tilt_state > -75.0){
								command_msg.tilt = tilt_state - 15.0;
							}else if(tilt_state<=-75.0 && tilt_state>-85.0){
								command_msg.tilt = -90.0;
							}else{
								command_msg.tilt = -10.0;
							}
						}else{
							command_msg.pan = pan_state + 15.0;
							command_msg.tilt = tilt_state;
						}
						command_pub.publish(command_msg);
						up_to_date = false;
					} else {
						ROS_INFO("Robot detected after search.");
					}
				}
				// Scan of the whole area independently from the detection of a robot
				else if (mode == std::string("scan")) {
						if (pan_state>90.0){
							command_msg.pan = -90.0;
							if (tilt_state < -20.0){
								command_msg.tilt = tilt_state + 20.0;
							}else if(tilt_state>=-20.0 && tilt_state<-5.0){
								command_msg.tilt = 0.0;
							}else{
								command_msg.tilt = -90.0;
							}
						}else{
							command_msg.pan = pan_state + 30.0;
							command_msg.tilt = tilt_state;
						}
						command_pub.publish(command_msg);
						up_to_date = false;
				} else {
					ROS_INFO("Error: Mode does not exist");
				}
			}
		}
		
		// Boolean function called by the rosservice to set the current mode to scan, search or track
		bool on_mode(tracker::Mode::Request& request, tracker::Mode::Response& response){
			mode = request.str;
			ROS_INFO("New mode is: %s",mode.c_str());
			return true;
		}
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"tracker");
    Tracker tr;
    ros::NodeHandle n_mode;
    ros::ServiceServer service_mode = n_mode.advertiseService<tracker::Mode::Request,tracker::Mode::Response>("mode", boost::bind(&Tracker::on_mode, &tr, _1, _2));

	ros::spin();
    return 0;
}
