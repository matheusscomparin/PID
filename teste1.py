import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math

V_Mat = [2018000309, 2016006869, 2017009838, 34219,  2017003253]

kp = 0.01
ki = 0.001
Int = 0
kd = 0.01
old_error = 0
estado = 1
kp_anda = 0.1
ki_anda = 0.07
kd_anda = 1
odom = Odometry()
scan = LaserScan()

rospy.init_node('cmd_node')

# Auxiliar functions ------------------------------------------------
def loop_timer_CallBack (V_Mat): 
    global mat
    n = len(V_Mat)
    res_each = 0
    m = 0
    f = 0
    t = 0
	
    for mat in V_Mat:
        res_each = 0
        for x in str (mat):
            res_each = res_each + int(x)
            m = m+res_each
    m = float (m)/n
    f = m
    t = 1/f
    return t
    
tempo_loop = loop_timer_CallBack(V_Mat)

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
    global estado
    error = 360
    msg = Twist()
    scan_len = len(scan.ranges)
    
    if not(scan_len > 0):
        control = 0
        msg.angular.z = 0
    
    else:
        if estado == 1:
            if min(scan.ranges[scan_len-10 : scan_len+10]) < 100:
                print('entrou no controle')
                msg.angular.z = 0
                yaw = getAngle(odom)
                dir_obj = min(scan.ranges[scan_len-10 : scan_len+10])
                
                setpoint = (dir_obj - scan.ranges[0])/(scan.ranges[scan_len-1] - scan.ranges[0]) #interpolacao
                setpoint *= 200 #interpolacao
                setpoint -= 100 #interpolacao
            
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
                Int += error*tempo_loop
                I = Int * ki
                D = delta_e * kd
                
                control = P+I+D
                if control > 1:
                    control = 1
                elif control < -1:
                    control = -1
                
            else:
                if min(scan.ranges[scan_len-15 : scan_len+15]) < 100: #se nao enncontrou o objeto roda ate achar
                    control = 0.15
                else:
                    control = 0.5
                print('rodando')
            
            
            msg.angular.z = control
            pub.publish(msg)
            
            if abs(error) < 1:
                Int = 0
                estado = 2
                print('SEGUNDO ESTADO')
                
        
        if estado == 2:
            print('COMECA ANDAR')
            read = min(scan.ranges)
            setpoint = 0.5 # 50 cm do objeto
            msg.angular.z = 0
            scan_len = len(scan.ranges)
            if scan_len > 0:
                real_dist = min(scan.ranges[scan_len-10 : scan_len+10])
                
                error = -(setpoint - real_dist)
                delta_e = error - old_error
                old_error = error
                 
                P = kp_anda*error
                Int += error*tempo_loop
                I = Int * ki_anda
                D = delta_e * kd_anda
                
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

timer = rospy.Timer(rospy.Duration(tempo_loop), timerCallBack)

rospy.spin()