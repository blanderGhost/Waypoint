#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
import time


from nav_msgs.msg import Odometry


def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        # return roll_x, pitch_y, yaw_z # in radians

        return yaw_z



def myCallback(pose_message):
    global x
    global y, yaw

    x = pose_message.pose.pose.position.x
    y = pose_message.pose.pose.position.y

    q1 = pose_message.pose.pose.orientation.x
    q2 = pose_message.pose.pose.orientation.y
    q3 = pose_message.pose.pose.orientation.z
    q4 = pose_message.pose.pose.orientation.w

    yaw = euler_from_quaternion(q1, q2, q3, q4)

 
def go_to_goalM(velocity_publisher, x_goal, y_goal, yaw_goal):
    global x
    global y, yaw

    velocity_message = Twist()


    while(True):
        K_angular = 2.5
        # print(y_goal-y)
        # print(x_goal-x)
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        error_angle = desired_angle_goal-yaw
        angular_speed = error_angle*K_angular
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)

        # print ('x=', x, ', y=',y, ', desired_angle_goal: ', desired_angle_goal) 
        # print(error_angle)
        if (abs(error_angle) < 0.004):
            
            velocity_message.angular.z = 0
            velocity_publisher.publish(velocity_message)
            break

    # print("angle acheived")
    
    while (True):
        
        K_linear = 0.5
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        linear_speed = distance * K_linear

        K_angular = 4
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)

       
        #     print(count)
        #     # print('desired_angle :', desired_angle_goal)
        #     # print('present angle = :' , yaw)
        #     print('error in angle = ', desired_angle_goal-yaw)
        #     print('error in distance = ', distance)

        error_angle = desired_angle_goal-yaw
        
        angular_speed = error_angle*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)

        # print (' position : x=', x, ', y=',y, ', distance to goal: ', distance)
        

        if (abs(distance) <0.1):
            velocity_message.linear.x = 0
            velocity_message.angular.z = 0

            velocity_publisher.publish(velocity_message)
            break
    
    # print("Position acheived")


    yaw_goal_rad = math.radians(yaw_goal)

    while(True):

        K_angular = 2.5
        
        # print('target_angle', yaw_goal_rad ,'present angle' , yaw)
        
        # desired_angle_goal = yaw_goal
        # error_angle = desired_angle_goal-yaw
        error_angle = yaw_goal_rad - yaw
        angular_speed = error_angle*K_angular

        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)

        # print (' final Angle : x=', x, ', y=',y, ', distance to goal: ', error_angle)
        # print('error ', error_angle)

        if ( abs(error_angle) < 0.008):
            velocity_message.linear.x = 0
            velocity_message.angular.z = 0

            velocity_publisher.publish(velocity_message)
            break

    # print("Angle  acheived")
    # print("error_angle", error_angle)
    # print(yaw*180/3.14)
    # print(yaw)

  



       



if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        cmd_vel_topic='/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    
        odom_sub = rospy.Subscriber('/odom', Odometry, myCallback)
        time.sleep(2)
        while(True):
            go_to_goalM(velocity_publisher, 10, 3, 50)
            go_to_goalM(velocity_publisher,-10 , 3 , 50)
            go_to_goalM(velocity_publisher, -10 ,-3, 50)
            go_to_goalM(velocity_publisher, 10, -3, 50)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")

