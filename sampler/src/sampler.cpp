#include <ros/ros.h>
#include <vector>
#include <array>
#include <detect/PanTilts.h>
#include <detect/PanTilt.h>
#include <iostream>
#include <fstream>
#include <std_srvs/Empty.h>


class Sampler {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber pantilt_sub;
        std::string pantilt_topic;
        double current_pan;
        double current_tilt;
        double current_x;
        double current_y;
        std::ifstream input;
        std::ofstream output;
        std::string path_to_input;
        std::string path_to_output;
        int detected_case;

    public:
		
        Sampler() : nh_("~") {
			nh_.param("pantilt_topic",pantilt_topic,std::string(""));
			nh_.param("path_to_input",path_to_input,std::string(""));
			nh_.param("path_to_output",path_to_output,std::string(""));
			// Creation of the subscriber
            pantilt_sub = nh_.subscribe(pantilt_topic,1,&Sampler::callback_pantilt,this);
            // Opening the file containing positions to sample
			input.open(path_to_input);
			output.open (path_to_output);
			if(!(!input)){
				input >> current_x >> current_y;
			} else {
				ROS_INFO("Cannot open file");
			}
			// Initialization of variable detected case arbitrarily
			// The variable allows to display changes concerning the number of robot detected once it occures and only when it occures
			detected_case = 2;
        }
					
        void callback_pantilt(const detect::PanTilts& msg){
			std::vector<detect::PanTilt> pantilt_vect = msg.pantilts;
			// One robot is detected
			// Pan tilt coordinates are stored
			if (pantilt_vect.size() == 1){
				current_pan = pantilt_vect[0].pan;
				current_tilt = pantilt_vect[0].tilt;
				if (detected_case!=0){
					ROS_INFO("One robot detected");
					detected_case = 0;
				}
			// No robot detected. No pan tilt can be stored.
			} else if ((pantilt_vect.size() == 0)&&(detected_case!=1)) {
				ROS_INFO("No robot is detected. No pan tilt position.");
				detected_case = 1;
			// Multiple robots detected. No pan tilt can be stored.
			} else if ((pantilt_vect.size() > 1)&&(detected_case!=2)){
				ROS_INFO("Multiple robots detected. Cannot choose a position.");
				detected_case = 2;
			}
		}
		
		// Function for rosservice allowing to register the current pan/tilt values in the position.txt file with the corresponding x,y positions from liste.txt file
		bool register_on(std_srvs::Empty::Request request,std_srvs::Empty::Response response){
			ROS_INFO("Following robot position registered: x = %f , y = %f",(double) current_x, (double) current_y);
			output << current_pan << " " << current_tilt << " " << current_x << " " << current_y << std::endl;
			// getting postition (x,y) from input
			if(!(input >> current_x >> current_y)){
				ROS_INFO("Closing position file");
				input.close();
				output.close();
				ros::shutdown();
			}
			return true;
		}	

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"detector");
    Sampler sam;
    ros::NodeHandle n_reg;
    ros::ServiceServer service_reg = n_reg.advertiseService<std_srvs::Empty::Request,std_srvs::Empty::Response>("register", boost::bind(&Sampler::register_on, &sam, _1, _2));
    
	ros::spin();

    return 0;
}
