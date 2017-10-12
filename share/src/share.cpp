#include <ros/ros.h>
#include <vector>
#include <detect/ArenaPosition.h>
#include <detect/ArenaPositions.h>
#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <iomanip>
#include <iterator>
#include <boost/asio.hpp>

class Share {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber positions_sub;
		std::string positions_topic;
		std::string hostname;
		std::string port;
		int port_nb;
		std::chrono::duration<int,std::milli> pause_duration;
		int duration_time;
		boost::asio::ip::tcp::iostream socket;
		int persistance;
		
    public:
		
        Share() : nh_("~") {
			nh_.param("positions_topic",positions_topic,std::string("/localize/ArenaPositions"));
			nh_.param("hostname",hostname,std::string(""));
			nh_.param("port_nb",port_nb,0);
			nh_.param("duration_time",duration_time,500);
			nh_.param("persistance",persistance,1);
            positions_sub = nh_.subscribe(positions_topic,1,&Share::callback_positions,this);
			pause_duration = std::chrono::duration<int,std::milli>(duration_time);
			
			// Connection to the server
			try {
				// Let us create a stream and handle it with exceptions.
				socket.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);
				
				// Convertion of port number to string port
				std::stringstream ss;
				ss << port_nb;
				port = ss.str();
				ROS_INFO("hostname %s, port %s", hostname.c_str(), port.c_str());
				
				// Let us connect the client socket.
				socket.connect(hostname,port);
				ROS_INFO("I set persistance to %s", (double) persistance);
				socket << "persistance " << persistance << std::endl;
			}
			catch(std::exception& e) {
				std::cerr << "Exception caught : " << e.what() << std::endl;
		    }
        }
        
		// Each times an arenapositions messages is received, the coordinates are sent to the server
        void callback_positions(const detect::ArenaPositions& msg){
			try {
				std::vector<detect::ArenaPosition> position_vect = msg.arenapositions;
				for (auto pos : position_vect){
					//envoyer la position au serveur
					socket << "put " << pos.x << ' ' << pos.y << std::endl;
					ROS_INFO("The point x= %f, y=%f has been sent", (double)pos.x, (double) pos.y);
					std::this_thread::sleep_for(pause_duration);
				}
			}
			catch(std::exception& e) {
				std::cerr << "Exception caught : " << e.what() << std::endl;
		    }
		}
		
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"share");
    Share sha;
	ros::spin();
    return 0;
}
