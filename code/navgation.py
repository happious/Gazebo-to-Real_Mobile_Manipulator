#!/usr/bin/env python

import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Point, PoseWithCovarianceStamped
from math import pi, sqrt, atan2
import csv

class PID:
    def __init__(self, P=0.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        self.error = self.set_point - current_value
        if self.error > pi:
            self.error -= 2 * pi
        elif self.error < -pi:
            self.error += 2 * pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator += self.error
        self.Integrator = max(self.Integrator_min, min(self.Integrator, self.Integrator_max))
        self.I_value = self.Integrator * self.Ki
        return self.P_value + self.I_value + self.D_value

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
        self.Integrator = 0

    def setPID(self, set_P=0.0, set_I=0.0, set_D=0.0):
        self.Kp = set_P
        self.Ki = set_I
        self.Kd = set_D

class turtlebot_move():
    def __init__(self):
        rospy.init_node('turtlebot_move', anonymous=False)
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.stop)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pid_theta = PID(0, 0, 0)
        self.is_moving = False
        self.target_queue = []

        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.point_sub = rospy.Subscriber("/robot_point", Point, self.point_callback)

        self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.vel = Twist()
        self.rate = rospy.Rate(10)
        self.trajectory = []
        self.theta_values = []
        self.coordinate_data = []
        self.pid_data = []

        rospy.Timer(rospy.Duration(0.1), self.process_target_queue)

        rospy.spin()

    def point_callback(self, msg):
        if self.is_moving:
            rospy.logwarn("🚫 Robot is moving. Ignoring new target.")
        else:
            rospy.loginfo("📍 New target received: x=%.3f, y=%.3f", msg.x, msg.y)
            self.target_queue.append((msg.x, msg.y))

    def process_target_queue(self, event):
        if not self.is_moving and self.target_queue:
            target = self.target_queue.pop(0)
            self.move_to_point(target[0], target[1])

    def move_to_point(self, x, y):
        self.is_moving = True
        diff_x = x - self.x
        diff_y = y - self.y
        direction_vector = np.array([diff_x, diff_y])
        direction_vector = direction_vector / np.linalg.norm(direction_vector)
        theta = atan2(diff_y, diff_x)

        self.pid_theta.setPID(1.0, 0.0, 0.0)
        self.pid_theta.setPoint(theta)
        rospy.loginfo("🔄 Rotating to theta: %.2f rad", theta)

        while not rospy.is_shutdown():
            angular = self.pid_theta.update(self.theta)
            angular = max(-0.2, min(angular, 0.2))
            if abs(angular) < 0.01:
                break
            self.vel.linear.x = 0.0
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        self.stop()
        self.pid_theta.setPID(1.0, 0.02, 0.2)
        self.pid_theta.setPoint(theta)

        rospy.loginfo("🚗 Moving to target point (%.2f, %.2f)", x, y)
        while not rospy.is_shutdown():
            diff_x = x - self.x
            diff_y = y - self.y
            vector = np.array([diff_x, diff_y])
            linear = np.dot(vector, direction_vector)
            linear = max(-0.2, min(linear, 0.2))

            angular = self.pid_theta.update(self.theta)
            angular = max(-0.2, min(angular, 0.2))

            if abs(linear) < 0.01 and abs(angular) < 0.01:
                break

            self.vel.linear.x = linear
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        self.stop()
        rospy.loginfo("✅ Reached target: x=%.3f, y=%.3f", x, y)
        self.is_moving = False

    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)

    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]
        self.trajectory.append([self.x, self.y])
        self.theta_values.append(self.theta)
        self.coordinate_data.append([self.x, self.y, self.theta])
        self.pid_data.append([self.pid_theta.Kp, self.pid_theta.Ki, self.pid_theta.Kd])

if __name__ == '__main__':
    try:
        turtlebot_move()
    except rospy.ROSInterruptException:
        pass
