import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import random
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
import numpy as np
import time

class Coordinate:
    def __init__(self,x,y) -> None:
        self.x = x
        self.y = y
    def setX(self,x):
        self.x = x
    def setY(self,y):
        self.y = y

class CubicBezierCurve:
    def __init__(self,c1,c2,c3,c4):
        self.P0 = Coordinate(c1.x,c1.y)
        self.P1 = Coordinate(c2.x,c2.y)
        self.P2 = Coordinate(c3.x,c3.y)
        self.P3 = Coordinate(c4.x,c4.y)
        self.t = 0
        self.step_size = 0.0001
        self.control_points_x = [c1.x,c2.x,c3.x,c4.x]
        self.control_points_y = [c1.y,c2.y,c3.y,c4.y]
        self.x_points=[]
        self.y_points=[]
        self.weights = [1,1,1.01,1]
        self.dx = 0.00
        self.dy = 0.00
        self.ros_path = Path()
        self.createEquations()
        self.ros_path = self.publishPath()


    
    def createEquations(self):
        # Equation Coefficiants for x
        self.t3_coeff_x = (-self.P0.x)+(3*self.P1.x)+(-3*self.P2.x)+(self.P3.x)
        self.t2_coeff_x = (3*self.P0.x)+(-6*self.P1.x)+(3*self.P2.x)
        self.t_coeff_x =  (-3*self.P0.x)+(3*self.P1.x)
        self.const_coeff_x = self.P0.x 

        # Equation Coefficiants for y
        self.t3_coeff_y = (-self.P0.y)+(3*self.P1.y)+(-3*self.P2.y)+(self.P3.y)
        self.t2_coeff_y = (3*self.P0.y)+(-6*self.P1.y)+(3*self.P2.y)
        self.t_coeff_y =  (-3*self.P0.y)+(3*self.P1.y)
        self.const_coeff_y = self.P0.y 

    def printCurve(self):
        print("x = {} t^3 + ({}) t^2 + ({}) t + ({})\n".format(self.t3_coeff_x,self.t2_coeff_x,self.t_coeff_x,self.const_coeff_x))
        print("y = {} t^3 + ({}) t^2 + ({}) t + ({})\n".format(self.t3_coeff_y,self.t2_coeff_y,self.t_coeff_y,self.const_coeff_y))

    def setCoeff(self,t):
        self.P0_coeff = (-(t**3)+3*(t**2)-3*t+1)*self.weights[0]
        self.P1_coeff = (3*(t**3)-6*(t**2)+3*t)*self.weights[1]
        self.P2_coeff = (-3*(t**3)+3*(t**2))*self.weights[2]
        self.P3_coeff = (t**3)*self.weights[3]

    def getXVal(self,t):
        self.setCoeff(t)
        return (self.P0.x*self.P0_coeff)+(self.P1.x*self.P1_coeff)+(self.P2.x*self.P2_coeff)+(self.P3.x*self.P3_coeff)

    def getYVal(self,t):
        self.setCoeff(t)
        return (self.P0.y*self.P0_coeff)+(self.P1.y*self.P1_coeff)+(self.P2.y*self.P2_coeff)+(self.P3.y*self.P3_coeff)
    
    def getPoint(self,t):
        x_cor = self.t3_coeff_x*(t**3)+self.t2_coeff_x*(t**2)+self.t_coeff_x*t+self.const_coeff_x
        y_cor = self.t3_coeff_y*(t**3)+self.t2_coeff_y*(t**2)+self.t_coeff_y*t+self.const_coeff_y
        print("For t = {},(x,y) = ({},{})\n".format(t,x_cor,y_cor))
        return Coordinate(x_cor,y_cor)
    
    def getGradient(self,t):
        dx_t = (self.t3_coeff_x*3*(t**2))+(self.t2_coeff_x*2*t)+(self.t_coeff_x)
        dy_t = (self.t3_coeff_y*3*(t**2))+(self.t2_coeff_y*2*t)+(self.t_coeff_y)
        if(dx_t!=0):
            return 90-math.atan2(dy_t/dx_t,1)*180/3.14
        else:
            return 90.0

    def getDerivative(self,t):
        self.dx = (self.t3_coeff_x*3*(t**2))+(self.t2_coeff_x*2*t)+(self.t_coeff_x)
        self.dy = (self.t3_coeff_y*3*(t**2))+(self.t2_coeff_y*2*t)+(self.t_coeff_y)
        return (self.dx,self.dy)
    
    def publishPath(self):
        for i in range(len(self.x_points)):
            PosePoint = PoseStamped()
            PosePoint.pose.position.x = self.x_points[i]
            PosePoint.pose.position.y = self.y_points[i]
            self.ros_path.poses.append(PosePoint)

    def plotPointsOnCurve(self):
        t=0
        while(t<=1):
            self.x_points.append(self.getXVal(t))
            self.y_points.append(self.getYVal(t))
                #print("for t = {} the x : {} and y : {}\n".format(t,self.getXVal(t),self.getYVal(t)))
            t+=self.step_size

    def getDistance(self,x1,y1,x2,y2):
        return math.sqrt((x2-x1)**2+(y2-y1)**2)

    def getClosestPoint(self,x,y):
        min_distance = 100
        min_x=0
        min_y=0
        for i in range(0,len(self.x_points)):
            dist = self.getDistance(self.x_points[i],x,self.y_points[i],y)
            if(dist<min_distance):
                (min_x,min_y) = (self.x_points[i],self.y_points[i])
        return (min_x,min_y)
    
    def getTValueForPoint(self,x,y):
        c10 = (x-self.control_points_x[1])*(y-self.control_points_y[0]) - (x-self.control_points_x[0])*(y-self.control_points_y[1])
        c20 = (x-self.control_points_x[2])*(y-self.control_points_y[0]) - (x-self.control_points_x[0])*(y-self.control_points_y[2])
        return c10/(c10-0.5*c20)

class CatmullRomBezierConverter:
    def __init__(self,c1,c2,c3,c4):
        self.P0 = Coordinate(c1.x,c1.y)
        self.P1 = Coordinate(c2.x,c2.y)
        self.P2 = Coordinate(c3.x,c3.y)
        self.P3 = Coordinate(c4.x,c4.y)
        self.tou = 1
    def BezierToCatmull(self):
        self.P0_prime = Coordinate(self.P3.x+6*(self.P0.x-self.P1.x),self.P3.y+6*(self.P0.y-self.P1.y))
        self.P1_prime = Coordinate(self.P0.x,self.P0.y)
        self.P2_prime = Coordinate(self.P3.x,self.P3.y)
        self.P3_prime = Coordinate(self.P0.x+6*(self.P3.x-self.P2.x),self.P0.y+6*(self.P3.y-self.P2.y))
        return (self.P0_prime,self.P1_prime,self.P2_prime,self.P3_prime)
    def CatmullToBezier(self):
        self.P1_prime = Coordinate(self.P1.x+(self.P2.x-self.P0.x)/(self.tou*6),self.P1.y+(self.P2.y-self.P0.y)/(self.tou*6))
        self.P0_prime = Coordinate(self.P1.x,self.P1.y)
        self.P3_prime = Coordinate(self.P2.x,self.P2.y)
        self.P2_prime = Coordinate(self.P2.x-(self.P3.x-self.P1.x)/(self.tou*6),self.P2.y-(self.P3.y-self.P1.y)/(self.tou*6))
        return (self.P0_prime,self.P1_prime,self.P2_prime,self.P3_prime)

class findPath:
    def __init__(self,goals,global_c1,global_c2) -> None:
        self.viapoints = goals
        self.setViaPoints(goals)
        self.n = len(self.viapoints)-1
        self.colors = ['red','green','blue','cyan','magenta','yellow'] 
        self.curves = []
        self.plot = plt
        self.global_c1 = global_c1
        self.global_c2 = global_c2
        self.ros_path = Path()
        self.ros_path.header.frame_id = "map"
        self.createPath()
        self.printPathsFound()

    def setViaPoints(self,goals):
        for i in range(len(goals)):
            self.viapoints[i] = Coordinate(goals[i][0],goals[i][1])

    def getCurve(self,start_viapoint_index,end_viapoint_index,type):
        try:
            obj = CatmullRomBezierConverter(self.viapoints[start_viapoint_index-1],self.viapoints[start_viapoint_index],self.viapoints[end_viapoint_index],self.viapoints[start_viapoint_index+1])
        except:
            obj = CatmullRomBezierConverter(self.global_c1,self.viapoints[start_viapoint_index],self.viapoints[end_viapoint_index],self.global_c2)
        (bcp1,bcp2,bcp3,bcp4) = obj.CatmullToBezier()
        if(type=="middle"):
            curve = CubicBezierCurve(bcp1,bcp2,bcp3,bcp4)
        elif(type=="start"):
            curve = CubicBezierCurve(bcp1,self.global_c1,bcp3,bcp4)
        elif(type=="end"):
            curve = CubicBezierCurve(bcp1,bcp2,self.global_c2,bcp4)
        curve.plotPointsOnCurve()
        for i in range(len(curve.x_points)):
            PosePoint = PoseStamped()
            PosePoint.pose.position.x = curve.x_points[i]
            PosePoint.pose.position.y = curve.y_points[i]
            self.ros_path.poses.append(PosePoint)
            #rospy.loginfo("Added {},{} to path".format(PosePoint.pose.position.x,PosePoint.pose.position.y))
        return curve

    def createPath(self):
        # For the initial point, we don't need to find the bezier coefficiants as we want the path parallel to the convex hull edge  
        self.curves.append(self.getCurve(0,1,"start"))
        for i in range(1,self.n-1):
            self.curves.append(self.getCurve(i,i+1,"middle"))
        # For the final point, we don't need to find the bezier coefficiants as we want the path parallel to the convex hull edge         
        self.curves.append(self.getCurve(self.n-1,self.n,"end"))
        
    def plotPath(self):
        for curve in self.curves:
            self.plot = self.plotPointsFromSet(curve.control_points_x,curve.control_points_y,self.colors[random.randint(0,5)])
            self.plot = self.plotCurveFromPoints(curve.x_points,curve.y_points)
        self.plot.show()

    def printPathsFound(self):
        for i in range(len(self.curves)):
            print("Equations for path {} between ({},{}) and ({},{}): \n".format(i+1,self.viapoints[i].x,self.viapoints[i].y,self.viapoints[i+1].x,self.viapoints[i+1].y))
            self.curves[i].printCurve()
            

    def plotCurveFromPoints(self,x_points,y_points):
        plt.scatter(x_points,y_points)
        return plt

    def plotPointsFromSet(self,control_points_x,control_points_y,colour):
        plt.plot(control_points_x, control_points_y, color=colour, marker='o')
        return plt

class Robot_Model:
    def __init__(self):
        self.R = np.array([[0.00,0.00],[0.00,0.00]])
        self.delta = np.array([[0.00,0.00],[0.00,0.00]])
        self.Vd = np.array([1])
        self.dv_dt = np.array([[1.00],[1.00]])
        self.Kp = np.array([[0.00,0.00],[0.00,0.00]])
        self.e = np.array([[0.00],[0.00]])
        self.tolerance = np.array([[0.00],[0.00]])
        self.u = np.array([[0.00],[0.00]])
        self.P = np.array([[0.00],[0.00]])
        self.Pd = np.array([[0.00],[0.00]])
        

    def assignPath(self,curve):
        self.path = curve
    
    def assignRotMat(self,theta):
        self.R = np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])

    def assignTolerance(self,E1,E2):
        self.tolerance = np.array([[E1],[E2]])
        self.delta = np.array([[1,-E2],[0,E1]])

    def setPropGain(self,P1,P2):
        self.Kp = np.array([[P1,0.00],[0.00,P2]])

    def setCurrentPose(self,x,y):
        self.P = np.array([[x],[y]])
    
    def setGoal(self,x,y):
        self.Pd = np.array([[x],[y]])
    
    def setNearestPointAsGoal(self):
        (x_goal,y_goal) = self.path.getClosestPoint(self.P[0],self.P[1])
        self.setGoal(x_goal,y_goal)

    def setDerivative(self):
        t = self.path.getTValueForPoint(self.Pd[0],self.Pd[1])
        (self.dv_dt[0],self.dv_dt[1]) = self.path.getDerivative(t)
    
    def setError(self,e1,e2):
        self.e = np.array([[e1],[e2]])
    
    def updateError(self):
        self.e = np.matmul(self.R.transpose(),np.add(np.subtract(self.P,self.Pd),self.tolerance))
    
    def setConstants(self,theta,E1,E2,P1,P2):
        self.assignRotMat(theta)
        self.assignTolerance(E1,E2)
        self.setPropGain(P1,P2)

    
    def getVelocity(self):
        self.u = np.matmul(self.delta.transpose(),np.matmul(self.R.transpose()*self.Vd,self.dv_dt)-np.matmul(self.Kp,self.e))
        #print(self.u)
        return self.u

        
class VelocityController:
    def __init__(self,path) -> None:
        self.path = path
        self.curve_index = 0
        self.goal_index=1
        #rospy.loginfo("going from : ({},{}) to ({},{})".format(self.curr_path.control_points_x[0],self.curr_path.control_points_y[0],self.curr_path.control_points_x[3],self.curr_path.control_points_y[3]))
        self.current_pose = Coordinate(0,0)
        self.odom_msg = Odometry()
        self.quadrant = 1
        self.vel = Twist()
        self.stop = Twist()
        self.model = Robot_Model()
        rospy.Subscriber("/imu", Imu,self.imu_callback)
        rospy.Subscriber("/odom",Odometry,self.odom_callback,queue_size = 1)
        self.vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.path_pub = rospy.Publisher("/spline_path",Path,queue_size=1)

    def setQuadrant(self,c1,c2):
        if(c2.x>c1.x and c2.y>c1.y):
            self.quadrant = 1
        elif(c2.x<c1.x and c2.y>c1.y):
            self.quadrant = 2
        elif(c2.x>c1.x and c2.y<c1.y):
            self.quadrant = 4
        elif(c2.x<c1.x and c2.y<c1.y):
            self.quadrant = 3
        else:
            self.quadrant = 1
        rospy.loginfo("quadrant : {}".format(self.quadrant))
        
        
    def correctGoalAngle(self,yaw):
        rospy.loginfo("goal yaw : {}".format(yaw))
        if(self.quadrant==3 or self.quadrant == 4):
            yaw = yaw-180
        rospy.loginfo("goal yaw after correction : {}".format(yaw))
        return yaw

    #def getTValue(self):
    #    c10 = (self.current_pose.x-self.curr_path.control_points_x[1])*(self.current_pose.y-self.curr_path.control_points_y[0]) - (self.current_pose.x-self.curr_path.control_points_x[0])*(self.current_pose.y-self.curr_path.control_points_y[1])
    #    c20 = (self.current_pose.x-self.curr_path.control_points_x[2])*(self.current_pose.y-self.curr_path.control_points_y[0]) - (self.current_pose.x-self.curr_path.control_points_x[0])*(self.current_pose.y-self.curr_path.control_points_y[2])
    #    return c10/(c10-0.5*c20)

    def imu_callback(self,imu_msg):
        #print(self.path.ros_path)
        self.path_pub.publish(self.path.ros_path)
        try:
            self.curr_path = self.path.curves[self.curve_index]
        except IndexError:
            rospy.logerr("Coudn't get index of curve")
        self.model.assignPath(self.curr_path)
        quaternion_list = [imu_msg.orientation.x, 
                imu_msg.orientation.y, 
                imu_msg.orientation.z, 
                imu_msg.orientation.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion_list)
        self.yaw *= 180/np.pi
        self.model.setConstants(self.yaw,0.001,0.001,0.001,0.001)
        rospy.loginfo("yaw : {}\n".format(self.yaw))

    def odom_callback(self,odom_msg):
        self.odom_msg = odom_msg
        self.current_pose.x = odom_msg.pose.pose.position.x
        self.current_pose.y = odom_msg.pose.pose.position.y
        self.model.setCurrentPose(self.current_pose.x,self.current_pose.y)
        #self.setQuadrant(Coordinate(self.current_pose.x,self.current_pose.y),Coordinate(self.curr_path.control_points_x[3],self.curr_path.control_points_y[3]))
        self.setQuadrant(Coordinate(self.curr_path.control_points_x[0],self.curr_path.control_points_y[0]),Coordinate(self.curr_path.control_points_x[3],self.curr_path.control_points_y[3]))
        #self.setQuadrant(Coordinate(0,0),Coordinate(self.curr_path.control_points_x[3],self.curr_path.control_points_y[3]))
        (curve_x,curve_y) = self.curr_path.getClosestPoint(self.current_pose.x,self.current_pose.y)
        self.curr_t = self.curr_path.getTValueForPoint(self.current_pose.x,self.current_pose.y)
        self.goal_yaw = self.correctGoalAngle(self.curr_path.getGradient(self.curr_t))
        self.model.setNearestPointAsGoal()
        self.model.setDerivative()
        self.model.updateError()
        self.model.getVelocity()
        #self.correctGoalAngle(self.curr_path.getGradient(self.curr_t))
        rospy.loginfo("goal yaw : {}\n".format(self.goal_yaw))
        self.theta_diff = self.goal_yaw-self.yaw
        rospy.loginfo("angle diff : {}\n".format(self.theta_diff))
        rospy.loginfo("going from : ({},{}) to ({},{}) along curve {}".format(self.curr_path.control_points_x[0],self.curr_path.control_points_y[0],self.curr_path.control_points_x[3],self.curr_path.control_points_y[3],self.curve_index))
        distance = math.sqrt((self.curr_path.control_points_x[3]-self.current_pose.x)**2+(self.curr_path.control_points_y[3]-self.current_pose.y)**2)
        rospy.loginfo("distance : {}\n".format(distance))
        self.setVelocity(distance)

    def setVelocity(self,distance):
        if(distance<1.5 and self.curve_index<=len(self.path.curves)+1):
            self.curve_index+=1
            print("Reached Viapoint")
            time.sleep(1)
        elif(distance>1.5 and self.curve_index<=len(self.path.curves)+1):
            self.vel.linear.x = distance/100#math.cos(distance)
            self.vel.angular.z = (self.theta_diff)/100#math.sin(self.theta_diff)
            #print(self.vel)
            self.vel_pub.publish(self.vel)
        elif(distance>1.5 or (self.curve_index>len(self.path.curves)+1)):
            self.vel_pub.publish(self.stop)
            print("Goal Reached")
        else:
            self.vel_pub.publish(self.stop)
            print("Error Occured")

class findViaPoints:
    def __init__(self,start,c1,c2,r,goal) -> None:
        self.start = Coordinate(start[0],start[1])
        self.goal = Coordinate(goal[0],goal[1])
        self.global_c1_angle = c1
        self.global_c2_angle = c2
        self.dist_start = goal[0]-start[0]
        self.dist_end = goal[1]-start[1]
        self.dist = r
        self.viapoint_factor = 0.5
        self.cp_factor = 0.6667
        self.getEndControlPoints()
        self.n = 1
        self.viapoints = []
        for i in range(0,self.n+2):
            self.viapoints.append((0,0))

    def getEndControlPoints(self):
        x_start = self.start.x + self.cp_factor*self.dist_start*math.cos(np.deg2rad(self.global_c1_angle))
        y_start = self.start.y + self.cp_factor*self.dist_start*math.sin(np.deg2rad(self.global_c1_angle))
        x_end = self.goal.x + self.cp_factor*self.dist_end*math.cos(np.deg2rad(self.global_c2_angle))
        y_end = self.goal.y + self.cp_factor*self.dist_end*math.sin(np.deg2rad(self.global_c2_angle))
        rospy.loginfo("Control Point P1 : ({},{})".format(x_start,y_start))
        rospy.loginfo("Control Point P2 : ({},{})".format(x_end,y_end))
        self.global_c1 = Coordinate(x_start,y_start)
        self.global_c2 = Coordinate(x_end,y_end)

    def generateViaPoints(self,type):
        if(type=="bezier"):
            curve = CubicBezierCurve(self.start,self.global_c1,self.global_c2,self.goal)
            t=0
            for i in range(0,self.n+1):
                point = curve.getPoint(t)
                self.viapoints[i] = (point.x,point.y)
                rospy.loginfo("Viapoint {} : ({},{})".format(i,self.viapoints[i][0],self.viapoints[i][1]))
                t = t+1/self.n
            self.printViaPoints()
        elif(type=="mean"):
            self.viapoints[0] = (self.start.x,self.start.y)
            self.viapoints[1] = (self.viapoint_factor*(self.goal.x+self.start.x),self.viapoint_factor*(self.goal.y+self.start.y))
            self.viapoints[2] = (self.goal.x,self.goal.y)
            self.printViaPoints()
        else:
            self.viapoints[0] = (self.start.x,self.start.y)
            x_point = float(input("Enter x coordinate of viapoint : "))
            y_point = float(input("Enter x coordinate of viapoint : "))
            self.viapoints[1] = (x_point,y_point)
            self.viapoints[2] = (self.goal.x,self.goal.y)
            self.printViaPoints()


    
    def printViaPoints(self):
        for i in range(0,len(self.viapoints)):
            rospy.loginfo("Viapoint {} : ({},{})".format(i,self.viapoints[i][0],self.viapoints[i][1]))
        
    def getViaPoints(self):
        return self.viapoints
    
if __name__ == "__main__":
    rospy.init_node('spline_controller')
    initial_heading = 0.00
    goal_heading = -90.00
    viapoint_obj = findViaPoints((3,0),initial_heading,goal_heading,1,(5,5))
    viapoint_obj.generateViaPoints("mean")
    viapoints = viapoint_obj.getViaPoints()
    #viapoints=[(4,0),(4.5,2.5),(5,5)]
    path = findPath(viapoints,viapoint_obj.global_c1,viapoint_obj.global_c2)
    VelocityController(path)
    path.plotPath()
    rospy.spin()
