#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math
from numpy import deg2rad, rad2deg

class Turtlesim:
    #def __init__(self):
        
#Initialize these variables to avoid errors if the Publisher runs first
#This Publisher is located in move_forward() function.
        # self.current_x = 0
        # self.current_y = 0
        # self.current_theta = 0
        

    def callback_fn(self,data):
        #The callback function is called when a subscriber is created. This function reads the values of the data in the topic subscribed to.
        
        self.current_x = data.x
        self.current_y = data.y
        self.current_theta = rad2deg(data.theta)

        #Original range of rotation was from 0 to 180 degrees, then from -180 degrees to 0 degrees (full cycle).
        #The code below changes the full cycle range to 0-360 degrees.
        if self.current_theta < 0:
            self.current_theta += 360

    
    


    def move_forward(self,target_distance):
        #an instance of the message we will use to send our data
        #msg = Twist()
        #msg.linear.x = 0
        #msg.angular.z = 0        
        
        initial_x = self.current_x
        initial_y = self.current_y

        error = target_distance
        rospy.loginfo("The current error is " + str(error))
        self.position_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

        while(error > 0.1):
            #The error here is calculated as the relative distance between the current position and the initial position
            error = target_distance - math.sqrt((self.current_x - initial_x)**2 + (self.current_y - initial_y)**2)

            rospy.loginfo("The current error is " + str(error))
            
            #Proportional controller (P)
            self.msg.linear.x = 0.3 * error
            self.position_pub.publish(self.msg)

        self.msg.linear.x = 0
        self.position_pub.publish(self.msg)       





    def rotate(self,target_angle): 
        ## ***NOTEEE: target_angle is input in degrees

        self.position_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(20)

        self.msg = Twist()
        self.msg.linear.x = 0
        self.msg.angular.z = 0

        Kr = 0.0005
        tolerance = 1

        ## Rotating
        error = abs(target_angle - self.current_theta)
        ## CCW or CW
        error_CW = error          
        error_CCW  = 360 - error     

        ## If error_CW is smaller, rotate CW
        if error_CW < error_CCW:
            Kr *= -1
        
        while not rospy.is_shutdown():
            
            if error < tolerance:
                break
            #The error here is calculated as the relative angle
            error = abs(target_angle - self.current_theta)
            rospy.loginfo("The current error is " + str(error))

            self.msg.angular.z = Kr*error
            self.position_pub.publish(self.msg)
            self.rate.sleep()

        #Arrived to target orientation
        self.msg.angular.z = 0
        self.position_pub.publish(self.msg)
    

    def go_to_goal(self,target_x,target_y):
    #Create a while loop that merges both rotation and translation.
    #Publish the linear/angular velocities at the same time

    #Initialization
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0

        ## Proportional Parameters
        Kr =  0.1

        #Publisher
        velocity_cmd_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
        velocity_cmd_pub.publish(msg)
        rate = rospy.Rate(10)

        #Calculating the hypotenus
        target_distance = math.sqrt((self.current_x - target_x)**2 + (self.current_y - target_y)**2)

        #Calculating the initial errors (position and angle)
        initial_x = self.current_x
        initial_y = self.current_y
        position_error = target_distance - math.sqrt((self.current_x - initial_x)**2 + (self.current_y - initial_y)**2)

        target_angle = rad2deg(math.atan(target_y/target_x))
        angle_error = target_angle - self.current_theta

        rospy.loginfo('The target angle is ' + str(target_angle) + ' degrees.')
        rospy.loginfo('The target distance is ' + str(target_distance))

        try: 
            while(position_error > 0.2):
                
                #To check which direction to rotate (CCW or CW)
                if angle_error > 0:
                    angle_CCW = angle_error
                    angle_CW = 360 - angle_CCW

                    if angle_CCW < angle_CW:
                        msg.angular.z = 0.1 * abs(angle_error)
                        msg.linear.x = 0.1 * abs(position_error)
                        velocity_cmd_pub.publish(msg)
                        rate.sleep()                     
                    
                    else:
                        msg.angular.z = -0.1 * abs(angle_error)
                        msg.linear.x = 0.1 * abs(position_error)
                        velocity_cmd_pub.publish(msg)
                        rate.sleep()

                elif angle_error <= 0:
                    angle_CCW = angle_error + 360
                    angle_CW = 360 - angle_CCW

                    if angle_CCW < angle_CW:
                        msg.angular.z = 0.1 * abs(angle_error)
                        msg.linear.x = 0.1 * abs(position_error)
                        velocity_cmd_pub.publish(msg)
                        rate.sleep()                     
                    
                    else:
                        msg.angular.z = -0.1 * abs(angle_error)
                        msg.linear.x = 0.1 * abs(position_error)
                        velocity_cmd_pub.publish(msg)
                        rate.sleep()
                                     


                #Publishing initial linear and angular velocities
                # msg.angular.z = 0.1 * abs(angle_error)
                # msg.linear.x = 0.1 * abs(position_error)
                # velocity_cmd_pub.publish(msg)
                # rate.sleep()                     

                #Calculating the errors (angle and position)
                
                #angle_error is calculated as the relative angle
                target_angle = rad2deg(math.atan((target_y - self.current_y)/(target_x - self.current_x)))                
                
                if target_angle < 0:
                    target_angle += 360

                #To take into account points that have X, Y coordinates that are less than the intial ones
                if target_y < self.current_y and target_x < self.current_x:
                    target_angle += 180

                angle_error = target_angle - self.current_theta
                rospy.loginfo("The target angle  is " + str(target_angle) + " degrees.")
                
                #position_error is the difference between the total distance needed to be covered and the current distance covered 
                position_error = math.sqrt((self.current_x - target_x)**2 + (self.current_y - target_y)**2)

                #position_error = target_distance - math.sqrt((self.current_x - initial_x)**2 + (self.current_y - initial_y)**2)
                rospy.loginfo("The current position error is " + str(position_error))

                if angle_error < 0.2:
                    break

        except:
            pass

        #Arrived to target angle and position
        msg.angular.z = 0
        msg.linear.x = 0
        velocity_cmd_pub.publish(msg)


        #def absolute(self, target_x, target_y):
            #This function is used to calculate the absolute distance and absolute angle needed to reach the target

            

        
          


def main():
    rospy.init_node('location_monitor')    

    instance = Turtlesim()

    #turtle1/pose is the topic we are subscribing to
    rospy.Subscriber("turtle1/pose", Pose, instance.callback_fn)

    time.sleep(1)
    #instance.move_forward(2.0)
    instance.rotate(270)
    # instance.go_to_goal(10,10)  
    # instance.go_to_goal(9,9)  

    #rospy.spin()
    #instance.rotate(90)
    #instance.go_to_goal(10,7)

if __name__ =='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass