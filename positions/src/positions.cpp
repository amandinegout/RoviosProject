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

class Positions {
    protected:
        ros::NodeHandle nh_;
        ros::Publisher positions_pub;
		std::string positions_server_topic;
		std::string hostname;
		std::string port;
		int port_nb;
		std::chrono::duration<int,std::milli> pause_duration;
		int duration_time;
		boost::asio::ip::tcp::iostream socket;
		int persistance;
		
    public:
		
        Positions() : nh_("~") {
			nh_.param("positions_server_topic",positions_server_topic,std::string(""));
			nh_.param("hostname",hostname,std::string(""));
			nh_.param("port_nb",port_nb,0);
			nh_.param("duration_time",duration_time,500);
			nh_.param("persistance",persistance,1);
            positions_pub = nh_.advertise<detect::ArenaPositions>(positions_server_topic,1);
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

		// Get the positions stored in the server and publishes them as an arenapositions messages
		void getAndPublish(){
			try {
				unsigned int nb;
				socket << "get" << std::endl;
				socket >> nb;
				ROS_INFO("The client got %d points", (int) nb);
				if (nb!=0){
					detect::ArenaPositions positions;
					for (unsigned int i=0;i<nb;i++){
						detect::ArenaPosition position;
						socket >> position.x >> position.y;
						positions.arenapositions.push_back(position);
					}
					positions_pub.publish(positions);
				} else {
					ROS_INFO("No points detected on server.");
				}
				std::this_thread::sleep_for(pause_duration);
			}
			catch(std::exception& e) {
				std::cerr << "Exception caught : " << e.what() << std::endl;
		    }
		}
		
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"positions");
    Positions pos;
    // While the node is active, it will continue reading the positions in the server and publishing them
    while (true) {
		pos.getAndPublish();
	}
    return 0;
}
