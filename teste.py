import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math

V_Mat = [2018000309, 2016006869, 2017009838, 34219,  2017003253]


kp_ang = 0.01
ki_ang = 0.0001
kd_ang = 0.01

kp = 1
ki = 0.01
kd = 1

odom = Odometry()
scan = LaserScan()

rospy.init_node('cmd_node')

# Auxiliar functions ------------------------------------------------

#FUNCAO PARA ACHAR A FREQUENCIA DO TIMER

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


# OBTER ANGULO EM GRAU
def getAngle(msg):
    quaternion = msg.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw = euler[2]*180.0/math.pi
    return yaw

# CALLBACKS --------------------------------------------------------
def odomCallBack(msg):
    global odom
    odom = msg
    
def scanCallBack(msg):
    global scan
    scan = msg
#--------------------------------------------------------------------

# TIMER - Control Loop ----------------------------------------------
def timerCallBack(event):
    global kp, ki, kd, kp_ang, ki_ang, kd_ang
#DEFINICAO DE VARIAVEIS    
    I_ang = 0
    I = 0
    msg = Twist()
# ERRO ANTERIOR INICIALIZADOS COMO NULO    
    p_erro_ang =0
    p_erro =0
    control_ang = 0

#ESTADO INICIAL BUSCA ANGULO QUE O ROBO DEVE IR    
    estado = 'Angulo'
    if estado == 'Angulo':
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
                    
            print(ang, yaw, error)
            
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
        
        if abs(error) < 1:
            Int = 0
            estado = 'Distancia'
         
           
       
                    
    if estado == 'Distancia':
        setpoint = 0.5 # 50 cm do objeto
        msg.angular.z = 0
        scan_len = len(scan.ranges)
        if scan_len > 0:
            real_dist = min(scan.ranges[scan_len-10 : scan_len+10])

            error = -(setpoint - real_dist)
        
            P = kp*error #parte proporcional
            I = I + ki*error  #parte integrativa
            D = (error - p_erro)*kd # parte derivativa
            
            control = P+I+D
            erro = p_erro #erro atual sera proximo antigo
                
            if control > 1:
                control = 1
            elif control < -1:
                control = -1
        else:
            control = 0        
        
       
        msg.linear.x = control
        
  
    pub.publish(msg)
    

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(tempo_loop), timerCallBack)

rospy.spin()
