import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math


kp = 0.01
ki = 0.0001
Int = 0
kd = 0.01
old_error = 0

T = 0.1

odom = Odometry()
scan = LaserScan()

rospy.init_node('cmd_node')

# Auxiliar functions ------------------------------------------------
def getAngle(msg):
    quaternion = msg.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw = euler[2]*180.0/math.pi
    return yaw

# CALLBACKS ---------------------------------------------------------
def odomCallBack(msg):
    global odom
    odom = msg
    
def scanCallBack(msg):
    global scan
    scan = msg
#--------------------------------------------------------------------

# TIMER - Control Loop ----------------------------------------------
def timerCallBack(event):
    global kp, ki, kd
    global Int, old_error
    estado = 1
    
    
    if estado == 1:
        setpoint = 0.5
        
        scan_len = len(scan.ranges)
        
        if scan_len > 0:
            yaw = getAngle(odom)
            
            ind = scan.ranges.index(min(scan.ranges))
            inc = 2*math.pi / scan_len
            ang = (ind * inc * 180.0/math.pi) + yaw
            if ang > 180:
                ang -= 360
                
            error = (ang - yaw)
            
            if abs(error) > 180:
                if setpoint < 0:
                    error += 360 
                else:
                    error -= 360
                    
            #print(ang, yaw, error)
            
            delta_e = error - old_error
            old_error = error
             
            P = kp*error
            Int += error*T
            I = Int * ki
            D = delta_e * kd
            
            control = P+I+D
            if control > 1:
                control = 1
            elif control < -1:
                control = -1
        else:
            control = 0        
        
        msg = Twist()
        msg.angular.z = control
        pub.publish(msg)
        
        if abs(error) < 100:
            print('SEGUNDO ESTADO')
            Int = 0
            estado = 2
            
    
    elif estado == 2:
        read = min(scan.ranges)
        setpoint = 0.5 # 50 cm do objeto
        msg.angular.z = 0
        scan_len = len(scan.ranges)
        if scan_len > 0:
            real_dist = min(scan.ranges[scan_len-10 : scan_len+10])
            
            error = -(setpoint - real_dist)
            delta_e = error - old_error
            old_error = error
             
            P = kp*error
            Int += error*T
            I = Int * ki
            D = delta_e * kd
            
            control = P+I+D
            if control > 1:
                control = 1
            elif control < -1:
                control = -1
        else:
            control = 0        
        
        
        msg = Twist()
        msg.linear.x = control
        pub.publish(msg)
        

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(T), timerCallBack)

rospy.spin()
