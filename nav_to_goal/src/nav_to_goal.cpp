#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <geometry_msgs/Pose2D.h>
#include <angles/angles.h>
#include <math.h>


class NavToGoal {
    protected:
        ros::NodeHandle nh_;
        ros::Publisher velPub;
        double max_angular_velocity;
        double distance_min;
        double k_vel;
        double vel_max;
        double k_alpha;
        std::string TARGET_TF;
        std::string SOURCE_TF;
        double goal_x;
        double goal_y;
        


    public:
		
        NavToGoal() : nh_("~") {
			nh_.param("max_angular_velocity",max_angular_velocity,1.0);
			nh_.param("distance_min",distance_min,0.1);
			nh_.param("k_vel",k_vel,5.0);
			nh_.param("vel_max",vel_max,2.0);
			nh_.param("k_alpha",k_alpha,3.0);
			nh_.param("TARGET_TF",TARGET_TF,std::string("turtle1"));
			nh_.param("SOURCE_TF",SOURCE_TF,std::string("world"));
			nh_.param("goal_x",goal_x,0.0);
			nh_.param("goal_y",goal_y,0.0);

			velPub = nh_.advertise<geometry_msgs::Twist>("nao_vel",1);
        }
        
		bool transformPose2D(const geometry_msgs::Pose2D& pose_src,
					 std::string source_frame,
					 geometry_msgs::Pose2D& pose_dst,
					 std::string target_frame) {
		  ros::Time now = ros::Time(0);
		  // Build up a geometry_msgs::PoseStamped from a geometry_msgs::Pose2D
		  geometry_msgs::PoseStamped pose_src_stamped;
		  pose_src_stamped.header.seq = 0;
		  pose_src_stamped.header.stamp = now;
		  pose_src_stamped.header.frame_id = source_frame;
		  pose_src_stamped.pose.position.x = pose_src.x;
		  pose_src_stamped.pose.position.y = pose_src.y;
		  pose_src_stamped.pose.position.z = 0;
		  pose_src_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose_src.theta);

		  tf::Stamped<tf::Pose> tf_pose_src, tf_pose_dst;
		  // Build up a tf::Stamped<Pose> from a geometry_msgs::PoseStamped
		  tf::poseStampedMsgToTF(pose_src_stamped, tf_pose_src);
		  
		  tf::TransformListener tf_listener;
		  try {
			// Let us wait for the frame transformation to be avaiable
			tf_listener.waitForTransform(source_frame, target_frame, now,  ros::Duration(1.0));
			
			// And compute the transformation
			tf_listener.transformPose(target_frame, tf_pose_src, tf_pose_dst);
		  }
		  catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
			return false;
		  }

		  double useless_pitch, useless_roll, yaw;
		  tf_pose_dst.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);
		  // move the yaw in [-pi, pi]
		  yaw = angles::normalize_angle(yaw);

		  // Transform the tf::Pose back into the Pose2D
		  pose_dst.x = tf_pose_dst.getOrigin().x();
		  pose_dst.y = tf_pose_dst.getOrigin().y();
		  pose_dst.theta = yaw;

		  return true;
		}
		
		// This function computes the cmd to send to move toward target_pose
		// The command rule used here is first to orientate the robot with a precision of pi/9
		// Then to move forward while correcting the orientation
		void moveTo(const geometry_msgs::Pose2D& target_pose, geometry_msgs::Twist& cmd_vel) {
			double distance = sqrt(pow(target_pose.x,2)+pow(target_pose.y,2));
			//calcul de r et v
			double alpha = atan2(target_pose.y,target_pose.x);
			if(fabs(alpha)>M_PI/9){
				cmd_vel.angular.z = ((alpha>0)?+1:-1)*max_angular_velocity;
				cmd_vel.linear.x = 0;	
			}else{
				cmd_vel.linear.x = std::min(k_vel*distance,vel_max);
				cmd_vel.angular.z = std::max(std::min(k_alpha*alpha,max_angular_velocity),-max_angular_velocity);	
			}
		}
		
		// During the movement, this function is called in a loop
		bool exploration (){
			geometry_msgs::Pose2D source_goal;
			source_goal.x=goal_x;
			source_goal.y=goal_y;
			source_goal.theta=0.0;
			geometry_msgs::Twist cmd_vel;
			geometry_msgs::Pose2D target_goal;
			// creation of target_goal
			transformPose2D(source_goal, SOURCE_TF, target_goal, TARGET_TF);
			double distance = sqrt(pow(target_goal.x,2)+pow(target_goal.y,2));
			if(distance>distance_min){
				// if the distance to the target over a threshold, then send a cmd message
				moveTo(target_goal, cmd_vel);
				velPub.publish(cmd_vel);
				transformPose2D(source_goal, SOURCE_TF, target_goal, TARGET_TF);
				distance = sqrt(pow(target_goal.x,2)+pow(target_goal.y,2));
				return true;
			}else{
				// if the distance to the target is under a threshold, the robot stop moving
				cmd_vel.angular.z = 0;
				cmd_vel.linear.x = 0;
				velPub.publish(cmd_vel);
				return false;
			}
		}		

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"nav_to_goal");
    NavToGoal ntg;
	bool running = true;
	// While the robot is not close enough to the target, running = true, exploration() is called
	while(running){
		running=ntg.exploration();
	}
	
    ros::spin();
    return 0;
}
