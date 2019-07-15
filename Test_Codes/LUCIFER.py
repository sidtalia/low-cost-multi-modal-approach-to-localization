#! /usr/bin/env python
import rospy
import numpy as np 
import math as m
import tf
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3,PoseStamped, Transform, TransformStamped
from sensor_msgs.msg import NavSatFix,Imu
from geometry_msgs.msg import TwistWithCovarianceStamped,Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import tf2_ros


def distancecalcy(X1,X2,Y1,Y2):
	return m.sqrt((X1-X2)**2 + (Y1-Y2)**2)

def anglecalcy(x1,x2,y1,y2):
	angle = m.atan2((y2-y1),(x2-x1))
	if(angle<0):
		angle += 2*m.pi
	return angle

def path_callback(data):
	global intX
	global intY
	global int_slope
	pose = data.poses[-1]
	intX = pose.pose.position.x
	intY = pose.pose.position.y
	orientation_q = pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(p,r,int_slope) = euler_from_quaternion(orientation_list)

def Curv(t,KX1,KX2,KX3,KY1,KY2,KY3):
	delX = t*t*KX1 + t*KX2 + KX3
	delY = t*t*KY1 + t*KY2 + KY3
	del2X = 2*t*KX1 + KX2
	del2Y = 2*t*KY1 + KY2
	denominator = delX*delX + delY*delY
	dummy = denominator
	denominator *= denominator*denominator
	denominator = m.sqrt(denominator)
	del3Y = 2*KY1
	del3X = 2*KX1
	second_denominator = denominator*dummy 
	dK = ((del3Y*delX - del3X*delY)/denominator) - (3*(delX*del2Y - del2X*delY)*(delX*del2X + delY*del2Y)/second_denominator)
	sub_term_1 = (delX*del2Y - del2X*delY)
	sub_term_2 = 2*(delX*del2X + delY*del2Y)
	third_denominator = m.fabs(second_denominator*dummy)
	sub_term_3 = (del3Y*delX - del3X*delY)
	sub_term_4 = 2*(del2X**2 + del2Y**2 + del3X*delX+del3Y*delY)
	sub_term_5 = - del3X*del2Y + del3Y*del2X
	term_1 = 3.75*(sub_term_1*(sub_term_2**2))/third_denominator
	term_2 = -3*(sub_term_3*sub_term_2)/second_denominator
	term_3 = -1.5*(sub_term_1*sub_term_4)/second_denominator
	term_4 = sub_term_5/denominator
	d2K = term_1 + term_2 + term_3 + term_4
	return dK,d2K

def check_range(x,i):
	if(i):
		if(x>1):
			return 1
		if(x<0.5):
			return 0.5
		return x
	if(x<0):
		return 0
	if(x>0.5):
		return 0.5
	return x

def C_from_K_t(t,KX1,KX2,KX3,KY1,KY2,KY3):
	delX = t*t*KX1 + t*KX2 + KX3
	delY = t*t*KY1 + t*KY2 + KY3
	del2X = 2*t*KX1 + KX2
	del2Y = 2*t*KY1 + KY2
	denominator = delX*delX + delY*delY
	dummy = denominator
	denominator *= denominator*denominator
	denominator = m.sqrt(denominator)
	Curvature = ((delX*del2Y) - (delY*del2X))
	Curvature /= denominator
	return Curvature

def get_Curvature(X1,Y1,X2,Y2,X3,Y3,X4,Y4,t):
	Px = np.array([X1,X2,X3,X4])
	Py = np.array([Y1,Y2,Y3,Y4])
	KX1 = 9*X2 + 3*X4 - 3*X1 - 9*X3
	KY1 = 9*Y2 + 3*Y4 - 3*Y1 - 9*Y3
	KX2 = 6*X1 - 12*X2 + 6*X3
	KY2 = 6*Y1 - 12*Y2 + 6*Y3
	KX3 = 3*(X2 - X1)
	KY3 = 3*(Y2 - Y1)

	#get curvature for a corresponding value of t
	next_Curvature = C_from_K_t(t[2],KX1,KX2,KX3,KY1,KY2,KY3)
 	#using newton rhapshody method to find best estimates for curvature
	for i in range(2):
		count = 0
		h =1
		for j in range(3):
			dk,d2K = Curv(t[i],KX1,KX2,KX3,KY1,KY2,KY3)
			if(j>=1):
				last_h = h
				h = dk/d2K
				if(h*last_h<0):
					h = cmp(h,0)*min(m.fabs(last_h/2),m.fabs(h))
			else:
				h = dk/d2K
			t[i] = t[i]-h
			t[i] = check_range(t[i],i)
	 

	Curvature = np.zeros(2)
	Curvature[0] = C_from_K_t(t[0],KX1,KX2,KX3,KY1,KY2,KY3)
	Curvature[1] = C_from_K_t(t[1],KX1,KX2,KX3,KY1,KY2,KY3)
	t = t[:2]
	T = np.array([(1-t)**3,3*t*(1-t)**2,3*t*t*(1-t),t**3])
	Bx = T[0]*Px[0] + T[1]*Px[1] + T[2]*Px[2] + T[3]*Px[3]
	By = T[0]*Py[0] + T[1]*Py[1] + T[2]*Py[2] + T[3]*Py[3]
	return Curvature,next_Curvature,Bx,By


# def get_Curvature(X1,Y1,X2,Y2,X3,Y3,X4,Y4,t):
# 	Px = np.array([X1,X2,X3,X4])
# 	Py = np.array([Y1,Y2,Y3,Y4])
# 	KX1 = 9*X2 + 3*X4 - 3*X1 - 9*X3
# 	KY1 = 9*Y2 + 3*Y4 - 3*Y1 - 9*Y3
# 	KX2 = 6*X1 - 12*X2 + 6*X3
# 	KY2 = 6*Y1 - 12*Y2 + 6*Y3
# 	KX3 = 3*(X2 - X1)
# 	KY3 = 3*(Y2 - Y1)

# 	delX = t*t*KX1 + t*KX2 + KX3
# 	delY = t*t*KY1 + t*KY2 + KY3
# 	del2X = 2*t*KX1 + KX2
# 	del2Y = 2*t*KY1 + KY2
# 	denominator = delX*delX + delY*delY
# 	dummy = denominator
# 	denominator *= denominator*denominator
# 	denominator = m.sqrt(denominator)
# 	next_Curvature = ((delX*del2Y) - (delY*del2X))
# 	next_Curvature /= denominator

# 	Curvature = np.zeros(100)
# 	dK = np.zeros(100)
# 	d2k = np.zeros(10)
# 	for i in range(100):
# 		t = i*0.01
# 		delX = t*t*KX1 + t*KX2 + KX3
# 		delY = t*t*KY1 + t*KY2 + KY3
# 		del2X = 2*t*KX1 + KX2
# 		del2Y = 2*t*KY1 + KY2
# 		denominator = delX*delX + delY*delY
# 		dummy = denominator
# 		denominator *= denominator*denominator
# 		denominator = m.sqrt(denominator)
# 		Curvature[i] = ((delX*del2Y) - (delY*del2X))
# 		Curvature[i] /= denominator
# 		dK[i] = Curvature[i]
# 		Curvature[i] = m.fabs(Curvature[i])

# 	peaks = np.where((Curvature[1:-1] > Curvature[0:-2]) * (Curvature[1:-1] > Curvature[2:]))[0] + 1
# 	Curvature = dK

# 	try:
# 		a = peaks[1]
# 		t = peaks*0.01
# 		T = np.array([(1-t)**3,3*t*(1-t)**2,3*t*t*(1-t),t**3])
# 		Bx = T[0]*Px[0] + T[1]*Px[1] + T[2]*Px[2] + T[3]*Px[3]
# 		By = T[0]*Py[0] + T[1]*Py[1] + T[2]*Py[2] + T[3]*Py[3]
# 		return Curvature[peaks],next_Curvature,Bx,By

# 	except:
# 		try:
# 			a = peaks[0]*0.01
# 			t = np.array([a,1-a])
# 			T = np.array([(1-t)**3,3*t*(1-t)**2,3*t*t*(1-t),t**3])
# 			Bx = T[0]*Px[0] + T[1]*Px[1] + T[2]*Px[2] + T[3]*Px[3]
# 			By = T[0]*Py[0] + T[1]*Py[1] + T[2]*Py[2] + T[3]*Py[3]
# 			peaks = np.array([peaks[0],peaks[0]])
# 			return Curvature[peaks],next_Curvature,Bx,By

# 		except:
# 			t = np.array([0.1,0.9])
# 			T = np.array([(1-t)**3,3*t*(1-t)**2,3*t*t*(1-t),t**3])
# 			Bx = T[0]*Px[0] + T[1]*Px[1] + T[2]*Px[2] + T[3]*Px[3]
# 			By = T[0]*Py[0] + T[1]*Py[1] + T[2]*Py[2] + T[3]*Py[3]
# 			peaks = np.array([10,90])
# 			return Curvature[peaks],next_Curvature,Bx,By

# print(get_Curvature(0,0,1,1,0,1,-1,0,0.1))


def get_T(V,X1,Y1,X2,Y2,X3,Y3,X4,Y4,DT):
	L1 = distancecalcy(Y1,Y2,X1,X2)
	L2 = distancecalcy(Y2,Y3,X2,X3)
	L3 = distancecalcy(Y3,Y4,X3,X4)
	L4 = distancecalcy(Y4,Y1,X4,X1)
	L = L1+L2+L3
	L = 0.5*(L+L4)
	parameter = DT*V/L
	t1 = 0.5*(L1/(L1+L2))
	t2 = 1 - 0.5*(L3/(L3+L2))
	retval = np.array([t1,t2,parameter])
	if(parameter>1):
		retval = np.array([t1,t2,1])
		return retval
	return retval

def calculate_Curvatures(V, X, Y, slope1, destX, destY, slope2):
	ACCEL_MAX = 5

	int1,int2 = get_Intermediate_Points( slope1, slope2, X, destX, Y, destY)
	t = get_T( V, X, Y, int1[0], int1[1], int2[0], int2[1], destX, destY, 2e-2)
	Curvature,next_Curvature,Bx,By = get_Curvature(X, Y, int1[0], int1[1], int2[0], int2[1], destX, destY, t)
	dX = distancecalcy( Y, By[0], X, Bx[0])
	if(dX<0.1):
		dX = 0.1
	K_f = Curvature[0]
	gap = distancecalcy(Bx[0],Bx[1],By[0],By[1])
	if(gap>0.1 and m.fabs(Curvature[0]) > 0.01 and m.fabs(Curvature[1])>0.01):
		gap_inverse = 1/gap
		K1_inv = 1/m.fabs(Curvature[0])
		sqrt_K1_K2_inv = 1/m.sqrt(m.fabs(Curvature[0]*Curvature[0]))
		condition = gap_inverse*(K1_inv - sqrt_K1_K2_inv)*V
		if(condition>1):
			K_f = Curvature[1]
			dX = distancecalcy( Y, By[1], X, Bx[1])
	V2 = m.sqrt(ACCEL_MAX/m.fabs(K_f))
	Cur_max = max(m.fabs(next_Curvature),0.01)
	V_max = m.sqrt(ACCEL_MAX/Cur_max)
	dV = V2 - V
	# print(dV)
	deceleration = (dV/dX)*V
	if(deceleration < -ACCEL_MAX/5):
		print(deceleration)
		V_max = V2	
	yaw_Rate = V*next_Curvature
	return V_max,yaw_Rate


def get_Intermediate_Points(slope1, slope2, X1, X2, Y1, Y2):
	int1 = np.zeros(2)
	int2 = np.zeros(2)
	d = distancecalcy(Y2,Y1,X2,X1)
	# if(d>50):
	# 	d = 50
	int1[0] = X1 + 0.4*m.cos(slope1)*d
	int1[1] = Y1 + 0.4*m.sin(slope1)*d
	int2[0] = X2 - 0.4*m.cos(slope2)*d
	int2[1] = Y2 - 0.4*m.sin(slope2)*d
	return int1,int2


waypoints = np.load('waypoints.npy')
point = 0
waypoint = waypoints[point]
destX = waypoint[0]
destY = waypoint[1]
dest_slope = waypoint[2]

def destination_callback(data):
	global destX
	global destY
	global dest_slope
	global intermediate
	global points
	destX = data.pose.position.x
	destY = data.pose.position.y
	orientation_q = data.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(p,r,dest_slope) = euler_from_quaternion(orientation_list)
	if(points>0):
		a = np.array([destX,destY,dest_slope])
		intermediate.append(a)
		points -= 1

def imu_callback(data):
	ax = data.linear_acceleration.x
	ay = data.linear_acceleration.y
	# print(m.sqrt(ax**2 + ay**2))

def planner_callback(data):
	global destX
	global destY
	global dest_slope
	global waypoints
	global point
	vx = data.twist.twist.linear.x
	vy = data.twist.twist.linear.y
	
	X = data.pose.pose.position.x
	Y = data.pose.pose.position.y

	orientation_q = data.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(p,r,slope) = euler_from_quaternion(orientation_list) 
	velocity = vx*m.cos(slope) + vy*m.sin(slope) #automatically takes care of the sign
	twist = Twist()
	twist.linear.x = 0
	twist.angular.z = 0
	if(destX!=None):
		twist.linear.x,twist.angular.z = calculate_Curvatures(velocity, X, Y, slope, destX, destY, dest_slope)
		d = distancecalcy(X,destX,Y,destY)
		if(d<5):
			try:
				point+=1
				waypoint = waypoints[point]
				destX = waypoint[0]
				destY = waypoint[1]
				dest_slope = waypoint[2]
				# print(destX,destY,dest_slope)
			except:
				destX = None
			#how do i announce that I have reached?
		# if(twist.linear.x>10):
		# 	twist.linear.x = 10
	cmd_vel_pub.publish(twist)


class OdometryNode:
    # Set publishers
    pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)

    def __init__(self):
        # init internals
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None

        # Set the update rate
        rospy.Timer(rospy.Duration(0.01), self.timer_callback) # 20hz

        self.tf_pub = tf2_ros.TransformBroadcaster()

        # Set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        # Find the index of the racecar
        try:
            arrayIndex = msg.name.index('dlive::base_link')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]
        self.last_recieved_stamp = rospy.Time.now()

    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return

        cmd = Odometry()
        cmd.header.stamp = self.last_recieved_stamp
        cmd.header.frame_id = 'odom'
        cmd.child_frame_id = 'base_link'
        cmd.pose.pose = self.last_received_pose
        cmd.twist.twist = self.last_received_twist
        self.pub_odom.publish(cmd)

        tf = TransformStamped(
            header=Header(
                frame_id=cmd.header.frame_id,
                stamp=cmd.header.stamp
            ),
            child_frame_id=cmd.child_frame_id,
            transform=Transform(
                translation=cmd.pose.pose.position,
                rotation=cmd.pose.pose.orientation
            )
        )

        self.tf_pub.sendTransform(tf)

if __name__ == '__main__':
	rospy.init_node('waypoint_follower', anonymous=True)
	node = OdometryNode()
	cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	rospy.Subscriber("odom",Odometry, planner_callback)
	rospy.Subscriber("move_base_simple/goal",PoseStamped,destination_callback)
	rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan",Path,path_callback)
	rospy.Subscriber("imu",Imu,imu_callback)
	rospy.spin()
