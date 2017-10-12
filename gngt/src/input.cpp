#include <ctime>
#include <cstdlib>
#include <cmath>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <detect/ArenaPositions.h>
#include <detect/ArenaPosition.h>

double uniform(double min, double max); 

double uniform(double min, double max) {
  return min + (max-min)*(std::rand()/(1.0+RAND_MAX));
}

// Computes the euclidean distance beetween a point and an arenaposition
double distance(const pcl::PointXYZ& pt1, const detect::ArenaPosition& pt2){
	double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    double eucl_dist = dx*dx+dy*dy;
    return eucl_dist;
}

// Outputs the arenaposition which is the closest to a given point
detect::ArenaPosition closest_point(std::vector<detect::ArenaPosition> data_positions,pcl::PointXYZ current_point){
	detect::ArenaPosition closest_pt;
	closest_pt.x = data_positions[0].x;
	closest_pt.y = data_positions[0].y;
	double distance_min = distance(current_point,closest_pt);
	for (unsigned int i=1; i< data_positions.size();i++){
		double current_distance = distance(current_point,data_positions[i]);
		if (current_distance < distance_min){
			distance_min = current_distance;
			closest_pt = data_positions[i];
		}
	}
	return closest_pt;
}

void build_pointcloud(ros::Publisher& pub, int nbSamples, double min_x, double max_x, double min_y, double max_y, double threshold, const detect::ArenaPositions::ConstPtr& msg){
	pcl::PointCloud<pcl::PointXYZ> data_1;
	pcl::PCLPointCloud2 data_2;
	data_1.clear();
	const std::vector<detect::ArenaPosition>& arenaPosition_vect = msg->arenapositions;
	
	// Add random points if they are close enough to actual positions
	for(unsigned int i=0; i < nbSamples; ++i) {
		pcl::PointXYZ pt;
		pt.x = uniform(min_x,max_x);
		pt.y = uniform(min_y,max_y);
		pt.z = 0;
		detect::ArenaPosition closest = closest_point(arenaPosition_vect,pt);
		if(distance(pt,closest)<=threshold){
			data_1.push_back(pt);
		}
	}
	// Publish a pointcloud (visible using rviz)
	pcl::toPCLPointCloud2 (data_1, data_2);
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(data_2,output);
	output.header.stamp    = ros::Time::now();
    output.header.frame_id = "/map"; // This is the default frame in RViz
	pub.publish(output);
}


int main(int argc, char **argv)
{
  std::srand(std::time(0));

  ros::init(argc, argv, "input");
  ros::NodeHandle n_("~");

  ros::Rate loop_rate(100);
  ros::Publisher pub;
  ros::Subscriber sub;
  
  // Definifiton of parameters
  int nbSamples;
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double threshold;
  n_.param("nbSamples",nbSamples,(int)20000);
  n_.param("min_x",min_x,(double)1.0);
  n_.param("max_x",max_x,(double)11.0);
  n_.param("min_y",min_y,(double)2.0);
  n_.param("max_y",max_y,(double)13.0);
  n_.param("threshold",threshold,0.2);
  
  // Definition of subscriber and publisher
  pub = n_.advertise<sensor_msgs::PointCloud2> ("input", 1);
  sub = n_.subscribe<detect::ArenaPositions>("arenapositions_from_positions", 1, 
									  boost::bind(build_pointcloud,
									  boost::ref(pub),
									  boost::ref(nbSamples),
									  boost::ref(min_x),
									  boost::ref(max_x),
									  boost::ref(min_y),
									  boost::ref(max_y),
									  boost::ref(threshold),
									  _1));
  ros::spin();
  return 0;
}
