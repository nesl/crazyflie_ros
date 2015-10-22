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
from pid import yawPID

class Controller:
    Idle = 0
    Automatic = 1
    TakingOff = 2
    Landing = 3
    CONTROLLER_PERIOD = 0.100
    THRUST_HOVER = 42000
    THRUST_MIN = 37000
    THRUST_MAX = 46000

    def __init__(self, frame):
        self.frame = frame
        self.pubNav = rospy.Publisher('quadrotor/cmd_vel', Twist, queue_size=1)
        self.listener = TransformListener()
        # PID = default, P, I, D, MIN, MAX, name
        self.pidX   = PID(0, 6, 0.25, 0, -10, 10, "x")
        self.pidY   = PID(0, -6, -0.25, -0, -10, 10, "y")
        self.pidYaw = yawPID(0, +0.25, 0.01, 0.01, -20.0, 20.0, "yaw")
        # asymPID = default, P+, P-, I+, I-, D, MIN, MAX, name
        #self.pidZ   = asymPID(self.THRUST_HOVER, 0, 0, 200, 50, 0, self.THRUST_MIN, self.THRUST_MAX, "z")
        self.pidZ   = PID(self.THRUST_HOVER, 1500, 300, 3000, 35000, 45000, "z") # Ki was 500
        self.pidZ.clampIntegral(-1000, 3000) # was 6000

        self.state = Controller.Idle
        self.goal = Pose()
        rospy.Subscriber("goal_cflie", Pose, self._poseChanged)
        rospy.Service("takeoff", std_srvs.srv.Empty, self._takeoff)
        rospy.Service("land", std_srvs.srv.Empty, self._land)
        rospy.Service("terminate", std_srvs.srv.Empty, self._terminate)

        self.thrust = 0

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

    def _terminate(self, req):
        rospy.loginfo("Terminating!")
        self.state = Controller.Idle

        # emergency landing
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.z = 0
        self.pubNav.publish(msg)

        # reset PID
        self.pidReset()
        self.thrust = 0

        return std_srvs.srv.EmptyResponse()

    def _land(self, req):
        rospy.loginfo("Landing requested!")
        self.goal.position.z = 0.0

        return std_srvs.srv.EmptyResponse()

    def run(self):
        self.thrust = 0
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

                if self.thrust >= self.THRUST_HOVER :
                    self.pidReset()
                    #self.pidZ.integral = thrust / self.pidZ.ki
                    self.goal.position.z = 1.60
                    self.goal.position.x = self.origin[0]
                    self.goal.position.y = self.origin[1]
                    self.state = Controller.Automatic
                    #thrust = 0
                    rospy.loginfo("CF swithing to auto")
                else:
                    self.thrust += 2000
                    if self.thrust > self.THRUST_HOVER:
                        self.thrust = self.THRUST_HOVER
                    msg = Twist()
                    msg.linear.z = self.thrust
                    self.pubNav.publish(msg)

            if self.state == Controller.Landing:
                self.goal.position.z = 0.15
                if position[2] <= 0.15:
                    self.state = Controller.Idle
                    msg = Twist()
                    self.pubNav.publish(msg)

            if self.state == Controller.Automatic or self.state == Controller.Landing:
                # transform target world coordinates into local coordinates
                # Transform( SOURCE, TARGET )
                r = self.getTransform("/world", self.frame)
                if r:
                    position, quaternion, t = r

                    # target CF-frame pose
                    targetWorld = PoseStamped()
                    targetWorld.header.stamp = t
                    targetWorld.header.frame_id = "/world"
                    targetWorld.pose = self.goal

                    targetDrone = self.listener.transformPose(self.frame, targetWorld)

                    quaternion = (
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w)
                    euler = tf.transformations.euler_from_quaternion(quaternion)
                    yaw = np.degrees(euler[2])
                    print "roll: %.1f, pitch: %.1f, yaw: %.1f" % (np.degrees(euler[0]), np.degrees(euler[1]), np.degrees(euler[2]))
                    #rospy.loginfo("Quad dz: %.2f, zp: %d, zi: %d, zd: %d", targetDrone.pose.position.z, self.pidZ.plast, self.pidZ.integral, self.pidZ.dlast)

                    msg = Twist()
                    # PID update: (value, targetValue)
                    msg.linear.x = self.pidX.update(0.0, targetDrone.pose.position.x)
                    msg.linear.y = self.pidY.update(0.0, targetDrone.pose.position.y)
                    msg.linear.z = self.pidZ.update(0.0, targetDrone.pose.position.z)
                    msg.angular.z = self.pidYaw.update(90.0, -yaw)
                    self.pubNav.publish(msg)

            if self.state == Controller.Idle:
                msg = Twist()
                self.pubNav.publish(msg)

            rospy.sleep(self.CONTROLLER_PERIOD)

if __name__ == '__main__':

    rospy.init_node('controller', anonymous=True)
    frame = rospy.get_param("~frame")
    controller = Controller(frame)
    controller.run()
