#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
from tf import TransformListener
from std_msgs.msg import Empty, Float32
from geometry_msgs.msg import Twist, PoseStamped, Pose
import std_srvs.srv
from pid import PID

class Controller:
    Idle = 0
    Automatic = 1
    TakingOff = 2
    Landing = 3

    def __init__(self, frame):
        self.frame = frame
        self.pubNav = rospy.Publisher('quadrotor/cmd_vel', Twist, queue_size=1)
        self.listener = TransformListener()
        self.pidX = PID(10, 0, 0.0, -10, 10, "x")
        self.pidY = PID(-10, -0, -0.0, -10, 10, "y")
        self.pidZ = PID(1000, 200.0, 100.0, 10000, 50000, "z")
        self.pidYaw = PID(-0.0, 0.0, 0.0, -200.0, 200.0, "yaw")
        self.state = Controller.Idle
        self.goal = Pose()
        rospy.Subscriber("goal_cflie", Pose, self._poseChanged)
        rospy.Service("takeoff", std_srvs.srv.Empty, self._takeoff)
        rospy.Service("land", std_srvs.srv.Empty, self._land)

        # origin
        self.origin = []

    def getTransform(self, source_frame, target_frame):
        now = rospy.Time.now()
        success = False
        if self.listener.canTransform(source_frame, target_frame, rospy.Time(0)):
            t = self.listener.getLatestCommonTime(source_frame, target_frame)
            if self.listener.canTransform(source_frame, target_frame, t):
                position, quaternion = self.listener.lookupTransform(source_frame, target_frame, t)
                success = True
            delta = (now - t).to_sec() * 1000 #ms
            if delta > 300:
                rospy.logwarn("Latency: %f ms.", delta)
                self.listener.clear()
                rospy.sleep(0.02)
        if success:
            return position, quaternion, t

    def pidReset(self):
        self.pidX.reset()
        self.pidZ.reset()
        self.pidZ.reset()
        self.pidYaw.reset()

    def _poseChanged(self, data):
        self.goal = data

    def _takeoff(self, req):
        rospy.loginfo("Takeoff requested!")
        self.state = Controller.TakingOff
        return std_srvs.srv.EmptyResponse()

    def _land(self, req):
        rospy.loginfo("Landing requested!")
        self.state = Controller.Landing
        return std_srvs.srv.EmptyResponse()

    def run(self):
        thrust = 0
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            # get transform
            r = self.getTransform("/world", self.frame)
            if not r:
                continue
            position, quaternion, t = r

            # handle taking off
            if self.state == Controller.TakingOff:
                # set origin
                if len(self.origin) == 0:
                    self.origin = position

                if (thrust > 40000 and position[2] > 1.50) or thrust > 50000:
                    self.pidReset()
                    self.pidZ.integral = thrust / self.pidZ.ki
                    self.goal.position.z = 1.70
                    self.goal.position.x = self.origin[0]
                    self.goal.position.y = self.origin[1]
                    self.goal.position
                    self.state = Controller.Automatic
                    thrust = 0
                else:
                    thrust += 100
                    msg = Twist()
                    msg.linear.z = thrust
                    self.pubNav.publish(msg)

            if self.state == Controller.Landing:
                self.goal.position.z = 0.05
                if position[2] <= 0.1:
                    self.state = Controller.Idle
                    msg = Twist()
                    self.pubNav.publish(msg)

            if self.state == Controller.Automatic or self.state == Controller.Landing:
                # transform target world coordinates into local coordinates
                r = self.getTransform("/world", self.frame)
                if r:
                    position, quaternion, t = r
                    targetWorld = PoseStamped()
                    targetWorld.header.stamp = t
                    targetWorld.header.frame_id = "world"
                    targetWorld.pose = self.goal

                    targetDrone = self.listener.transformPose(self.frame, targetWorld)

                    quaternion = (
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w)
                    euler = tf.transformations.euler_from_quaternion(quaternion)

                    msg = Twist()
                    msg.linear.x = self.pidX.update(0.0, targetDrone.pose.position.x)
                    msg.linear.y = self.pidY.update(0.0, targetDrone.pose.position.y)
                    msg.linear.z = self.pidZ.update(0.0, targetDrone.pose.position.z)
                    msg.angular.z = self.pidYaw.update(0.0, euler[2])
                    self.pubNav.publish(msg)

            if self.state == Controller.Idle:
                msg = Twist()
                self.pubNav.publish(msg)

            rospy.sleep(0.01)

if __name__ == '__main__':

    rospy.init_node('controller', anonymous=True)
    frame = rospy.get_param("~frame")
    controller = Controller(frame)
    controller.run()
