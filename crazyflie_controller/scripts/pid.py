#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

class PID:
    def __init__(self, nominal, kp, ki, kd, minOutput, maxOutput, name):
        self.nom = nominal
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.minOutput = minOutput
        self.maxOutput = maxOutput
        self.integral = 0.0
        self.integral_clamp = (minOutput, maxOutput)
        self.dErrAvg = 0.0
        self.previousError = 0.0
        self.previousTime = rospy.get_time()
        self.pubOutput = rospy.Publisher('pid/output/' + name, Float32, queue_size=1)
        self.pubError = rospy.Publisher('pid/error/' + name, Float32, queue_size=1)
        self.pubP = rospy.Publisher('pid/p/' + name, Float32, queue_size=1)
        self.pubD = rospy.Publisher('pid/d/' + name, Float32, queue_size=1)
        self.pubI = rospy.Publisher('pid/i/' + name, Float32, queue_size=1)
        self.plast = 0
        self.dlast = 0

    def reset(self):
        self.integral = 0.0
        self.previousError = 0.0
        self.previousTime = rospy.get_time()

    def clampIntegral(self, minInt, maxInt):
        self.integral_clamp = (minInt, maxInt);

    def update(self, value, targetValue):
        time = rospy.get_time()
        dt = time - self.previousTime
        error = targetValue - value
        derror = (error - self.previousError)
        self.dErrAvg = (1-0.20)*self.dErrAvg + (0.20)*derror
        self.integral += self.ki * error * dt
        if self.integral > self.integral_clamp[1]: 
            self.integral = self.integral_clamp[1]
        if self.integral < self.integral_clamp[0]:
            self.integral = self.integral_clamp[0]
        p = self.kp * error
        self.plast = p
        d = 0
        if dt > 0:
            d = self.kd * (self.dErrAvg) / dt
        self.dlast = d
        i = self.integral
        output = self.nom + p + d + i
        self.previousError = error
        self.previousTime = time
        self.pubOutput.publish(output)
        self.pubError.publish(error)
        self.pubP.publish(p)
        self.pubD.publish(d)
        self.pubI.publish(i)
        return max(min(output, self.maxOutput), self.minOutput)


class yawPID:
    def __init__(self, nominal, kp, ki, kd, minOutput, maxOutput, name):
        self.nom = nominal
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.minOutput = minOutput
        self.maxOutput = maxOutput
        self.integral = 0.0
        self.integral_clamp = (minOutput, maxOutput)
        self.dErrAvg = 0.0
        self.previousError = 0.0
        self.previousTime = rospy.get_time()
        self.pubOutput = rospy.Publisher('pid/output/' + name, Float32, queue_size=1)
        self.pubError = rospy.Publisher('pid/error/' + name, Float32, queue_size=1)
        self.pubP = rospy.Publisher('pid/p/' + name, Float32, queue_size=1)
        self.pubD = rospy.Publisher('pid/d/' + name, Float32, queue_size=1)
        self.pubI = rospy.Publisher('pid/i/' + name, Float32, queue_size=1)
        self.plast = 0
        self.dlast = 0

    def reset(self):
        self.integral = 0.0
        self.previousError = 0.0
        self.previousTime = rospy.get_time()

    def clampIntegral(self, minInt, maxInt):
        self.integral_clamp = (minInt, maxInt);

    def update(self, value, targetValue):
        time = rospy.get_time()
        dt = time - self.previousTime
        # error based on shortest arc-length
        err1 = targetValue - value
        error = (err1 + 180) % 360 - 180

        derror = (error - self.previousError)
        self.dErrAvg = (1-0.20)*self.dErrAvg + (0.20)*derror
        self.integral += self.ki * error * dt
        if self.integral > self.integral_clamp[1]: 
            self.integral = self.integral_clamp[1]
        if self.integral < self.integral_clamp[0]:
            self.integral = self.integral_clamp[0]
        p = self.kp * error
        self.plast = p
        d = 0
        if dt > 0:
            d = self.kd * (self.dErrAvg) / dt
        self.dlast = d
        i = self.integral
        output = self.nom + p + d + i
        self.previousError = error
        self.previousTime = time
        self.pubOutput.publish(output)
        self.pubError.publish(error)
        self.pubP.publish(p)
        self.pubD.publish(d)
        self.pubI.publish(i)
        return max(min(output, self.maxOutput), self.minOutput)