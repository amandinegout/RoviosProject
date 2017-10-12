#!/usr/bin/env python
import roslib; roslib.load_manifest('kalman')
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
import numpy
from numpy.linalg import inv
import message_filters
import math
from detect.msg import Entity
from detect.msg import Entities
from geometry_msgs.msg import Twist

class KFExample:
    def __init__(self,name):
        self.name = name
        self.position_stddev = 0.50 # [m]
        self.ref_frame = "/map"
        rospy.init_node('kf_example')
        self.position_stddev = rospy.get_param("~position_stddev_m",self.position_stddev)
        rospy.loginfo("Starting KF Example" )
        self.array = Float64MultiArray()
        self.array.layout = MultiArrayLayout()
        self.array.layout.data_offset = 0
        self.array.layout.dim = [MultiArrayDimension('data',3,3)]
        self.array.data = [0.0] * 3
        
        self.array2 = Float64MultiArray()
        self.array2.layout = MultiArrayLayout()
        self.array2.layout.data_offset = 0
        self.array2.layout.dim = [MultiArrayDimension('measure',2,2)]
        self.array2.data = [0.0] * 2
        
        # Initialisation of the matrices
        self.A = numpy.mat([
            [1,0,0],
            [0,1,0],
            [0,0,1]])
        self.P = numpy.mat([
			[1,0,0],
			[0,1,0],
			[0,0,3.5]])
        self.Q = numpy.mat([
			[pow(0.5,2),0,0],
			[0,pow(0.5,2),0],
			[0,0,0.001]])
        self.H = numpy.mat([
			[1,0,0],
			[0,1,0]])
        self.R = numpy.mat([
			[0.1,0],
			[0,0.1]])
        self.U = numpy.vstack([0.0,0.0])
		
		# State is x, y, theta
        self.state = numpy.vstack([0.0, 0.0, 0.0])
        self.last_update = rospy.Time.now()
        
        
    def run(self):
        rate = rospy.Rate(5)
        self.pub = rospy.Publisher("~state",Float64MultiArray,queue_size=1)
        self.pub2 = rospy.Publisher("~measure",Float64MultiArray,queue_size=1)
        self.sub = rospy.Subscriber("/entities", Entities, self.callback_pos)
        self.sub_cmd = rospy.Subscriber("/rovio/cmd_vel", Twist, self.callback_cmd)
        # Now just wait...
        while not rospy.is_shutdown():
            rate.sleep()
            
    def callback_pos(self, data):
		#Prediction step
		#Calcul de dt
		t=rospy.Time.now()
		dt=0
		if t.nsecs-self.last_update.nsecs>0:
			dt=(t.nsecs-self.last_update.nsecs)*pow(10,-9)
		else:
			dt=(pow(10,9)+t.nsecs-self.last_update.nsecs)*pow(10,-9)
		
		
		
		self.A = numpy.mat([
			[1,0,-self.U[0,0]*dt*math.sin(self.state[2,0])],
			[0,1,self.U[0,0]*dt*math.cos(self.state[2,0])],
			[0,0,1]])
		
		B = numpy.mat([
			[dt*math.cos(self.state[2,0]),0],
			[dt*math.sin(self.state[2,0]),0],
			[0,dt]])
			
		self.state = self.A * self.state + B * self.U
		self.P=self.A*self.P*self.A.transpose()+self.Q*pow(dt,2)
		
		
		# Update step
		Z = numpy.vstack([data.entities[0].arenaposition.x,data.entities[0].arenaposition.y])
		innov=Z-self.H*self.state
		innov_cov=self.H*self.P*self.H.transpose()+self.R
		K=self.P*self.H.transpose()*inv(innov_cov)
		I = numpy.eye(3)
		self.state = self.state + K*innov
		self.P=(I-K*self.H)*self.P
		
        # Now publish the result
		self.array.data = [self.state[i] for i in range(3)]
		self.array.data[2] = self.array.data[2]*180/math.pi
		self.pub.publish(self.array)
		
		self.array2.data = [Z[i] for i in range(2)]
		self.pub2.publish(self.array2)
		
		self.last_update=rospy.Time.now()

    # Updates the command everytime a new command is published
    def callback_cmd(self, data):
		self.U = numpy.vstack([data.linear.x,data.angular.z])
		
		
if __name__ == '__main__':
    try:
        rd = KFExample("kalman_rovio") 
        rd.run()
    except rospy.ROSInterruptException:
        pass
        
