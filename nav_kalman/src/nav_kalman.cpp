#include <ros/ros.h>
#include <time.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <angles/angles.h>
#include <math.h>
#include <fstream>
#include <std_srvs/Empty.h>
#include "std_msgs/Float64MultiArray.h"

bool running;

class NavKalman {
    protected:
        ros::NodeHandle nh_;
        ros::Publisher velPub;
        double max_angular_velocity;
        double distance_min;
        double k_vel;
        double vel_max;
        double k_alpha;
        double pos_x;
        double pos_y;
        double pos_theta;
        geometry_msgs::Pose2D source_goal;
        geometry_msgs::PoseStamped source_goal_stamped;
        geometry_msgs::PoseArray trajectoire;
        ros::Publisher poseArrayPub;
        ros::Publisher posePub;
        ros::Subscriber posSub;
        uint8_t i;
        std::string SOURCE_FRAME;
        bool up_to_date;

    public:
		
        NavKalman() : nh_("~") {
			nh_.param("max_angular_velocity",max_angular_velocity,1.0);
			nh_.param("distance_min",distance_min,0.1);
			nh_.param("k_vel",k_vel,5.0);
			nh_.param("vel_max",vel_max,2.0);
			nh_.param("k_alpha",k_alpha,3.0);
			nh_.param("SOURCE_FRAME",SOURCE_FRAME,std::string("Room frame"));
			
			posSub = nh_.subscribe("/kalman_rovio/state", 1, &NavKalman::callback, this);
			velPub = nh_.advertise<geometry_msgs::Twist>("nao_vel",1);
			posePub = nh_.advertise<geometry_msgs::PoseStamped>("target",1);
			poseArrayPub = nh_.advertise<geometry_msgs::PoseArray>("trajectoire",1);
			
			// Gathering trajectories from file 
			// Creation of trajectoire message to publish the trajectories
			trajectoire.header.stamp = ros::Time::now();
			trajectoire.header.frame_id = SOURCE_FRAME;
			std::ifstream f;
			f.open("/usr/users/promo2016/lifchitz_yan/catkin_ws/src/nav_kalman/src/liste.txt");
			unsigned int indice = 0;
			while(!(!f)){
				geometry_msgs::PoseStamped pose;
				// Adding pose to pose array
				f >> pose.pose.position.x >> pose.pose.position.y;
				pose.pose.position.z = 0.0;
				ROS_INFO("x = %f , y = %f",(double) pose.pose.position.x, (double) pose.pose.position.y);
				pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
				// Computation of the angle between a position and the next one
				if(indice!=0){
					ROS_INFO("calcul angle");
					double angle = 0.0;
					angle = atan2((trajectoire.poses[indice].position.y-trajectoire.poses[indice-1].position.y),(trajectoire.poses[indice].position.x-trajectoire.poses[indice-1].position.x));
					ROS_INFO("angle = %f",(double) angle);
					trajectoire.poses[indice-1].orientation = tf::createQuaternionMsgFromYaw(angle);
				}
				trajectoire.poses.push_back(pose.pose);
				++indice;
			}
			poseArrayPub.publish(trajectoire);
			// Initialization of the index of target position to follow
			i = 0;
			// Creation of a Pose 2D message in the source frame (from the file).
			// Necessary to create a PoseStamped message first 
			source_goal_stamped.header = trajectoire.header;
			source_goal_stamped.pose = trajectoire.poses[i];
			source_goal.x=source_goal_stamped.pose.position.x;
			source_goal.y=source_goal_stamped.pose.position.y;
			ROS_INFO("Goal: x = %f , y = %f",(double) source_goal.x, (double) source_goal.y);
			source_goal.theta = 0.0;
			up_to_date = false;
        }
        
        void callback(const std_msgs::Float64MultiArray& msg){
			// Position of the robot returned by the Kalman filter and the server of positions
			pos_x = msg.data[0];
			pos_y = msg.data[1];
			pos_theta = msg.data[2];
			ROS_INFO("Current position from Kalman updated");
			// As Kalman publications are less frequent, limitation of moveTo callback to 
			// moments when current position returned by Kalman is up to date
			up_to_date = true;
		}
		
		void convertPosition(const geometry_msgs::Pose2D& pose_src, geometry_msgs::Pose2D& pose_dst){
			// Conversion of position from the room chosen frame to the robot frame
			ROS_INFO("Position of rovio in room frame: x=%f, y=%f, theta=%f.", (double)pos_x, (double)pos_y, (double)pos_theta);
			ROS_INFO("Position of target in room frame: x=%f, y=%f.", (double)pose_src.x, (double)pose_src.y);
			pos_theta = pos_theta*M_PI/180;
			pose_dst.x = (pose_src.x - pos_x)*cos(pos_theta)+(pose_src.y - pos_y)*sin(pos_theta);
			pose_dst.y = -(pose_src.x - pos_x)*sin(pos_theta)+(pose_src.y - pos_y)*cos(pos_theta);
			pose_dst.theta = atan2(pose_dst.y,pose_dst.x);
			ROS_INFO("Target position in rovio frame is : %f, %f, %f",(double)pose_dst.x,(double)pose_dst.y,(double)pose_dst.theta);
		}
		
		
		void moveTo(const geometry_msgs::Pose2D& target_pose, geometry_msgs::Twist& cmd_vel) {
			double distance = sqrt(pow(target_pose.x,2)+pow(target_pose.y,2));
			// Computation of linear and angular commands
			double alpha = atan2(target_pose.y,target_pose.x);
			if(fabs(alpha)>M_PI/18){
				ROS_INFO("Angle to rotate toward the target %f", (double) alpha);
				cmd_vel.angular.z = ((alpha>0)?+1:-1)*max_angular_velocity;
				cmd_vel.linear.x = 0;	
			}else{
				// Rovio cannot handle progressive commands on linear coordinates. 
				//Values limited between 0.22 and 0.3 according to documentation
				//     cmd_vel.linear.x = std::min(k_vel*distance,vel_max);
				cmd_vel.linear.x = vel_max;
				// Rovio cannot handle the command on linear and angular coordinates at the same time:
				//     cmd_vel.angular.z = std::max(std::min(k_alpha*alpha,max_angular_velocity),-max_angular_velocity);	
			}
		}
		
		// Exploration regarding the targets list
		bool exploration (){
			if(up_to_date){
				geometry_msgs::Twist cmd_vel;
				geometry_msgs::Pose2D target_goal;
				convertPosition(source_goal,target_goal);
				double distance = sqrt(pow(target_goal.x,2)+pow(target_goal.y,2));
				// Robot is still far from the target. It is asked to move to it.
				if(distance>distance_min){
					moveTo(target_goal, cmd_vel);
					velPub.publish(cmd_vel);
					up_to_date = false;
					posePub.publish(source_goal_stamped);
					poseArrayPub.publish(trajectoire);
					return true;
				}else{
					// Robot is now close enough to the target. It is asked to stop.
					cmd_vel.angular.z = 0;
					cmd_vel.linear.x = 0;
					velPub.publish(cmd_vel);
					up_to_date = false;
					if(i != trajectoire.poses.size() - 1){
						i++;
						source_goal_stamped = geometry_msgs::PoseStamped();
						source_goal_stamped.header = trajectoire.header;
						source_goal_stamped.pose = trajectoire.poses[i];
						source_goal.x=source_goal_stamped.pose.position.x;
						source_goal.y=source_goal_stamped.pose.position.y;
						ROS_INFO("goal: x = %f , y = %f",(double) source_goal.x, (double) source_goal.y);
						posePub.publish(source_goal_stamped);
						return true;
					}else{
						return false;
					}
				}
			}
		}
		// Initialization method called before exploration to set the Kalman filter on a simple trajectory
		void initialize(){
			geometry_msgs::Twist cmd_vel;
			for (unsigned int i = 0; i<10 ; i++){
				cmd_vel.linear.x = 0.3;
				velPub.publish(cmd_vel);
				ros::Duration(4.0).sleep();
				cmd_vel.linear.x = -0.3;
				velPub.publish(cmd_vel);
				ros::Duration(4.0).sleep();
			}
			cmd_vel.linear.x = 0.0;
			velPub.publish(cmd_vel);
		}
		
};

NavKalman* nk;

// Boolean function for rosservice go
bool go(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
	running=true;
	return true;
}

// Boolean function for rosservice abort
bool abort(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	running=false;
	return true;
}		


int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"nav_to_goal");
    nk = new NavKalman();
    ros::NodeHandle n_go;
    ros::NodeHandle n_abort;
    running = false;
    
    // Initialization of abort and go rosservices
    ros::ServiceServer service_go = n_go.advertiseService("go", go);
    ros::ServiceServer service_abort = n_abort.advertiseService("abort", abort);
    
	// Waiting for the go service to be called
	while(!running) {
		ros::spinOnce();
	}
	// Initialize method aims at lettinf the Kalman filter stabilize before running the exploration
	nk->initialize();
	// When go service is called, exploration starts
	
	while(ros::ok() && running) {
		running = nk->exploration();
		ros::spinOnce();
	}
    
    return 0;
}
