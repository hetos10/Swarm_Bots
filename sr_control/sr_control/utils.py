#!/usr/bin/env python3
"""
Utility functions for warehouse control
Includes kinematics, math utilities, and ROS message helpers
"""

import math
from geometry_msgs.msg import Twist, Vector3


class MecanumKinematics:
    """Mecanum wheel kinematics calculations"""
    
    def __init__(self, wheel_radius=0.05, chassis_length=0.40, chassis_width=0.40):
        """Initialize kinematics parameters"""
        self.wheel_radius = wheel_radius
        self.chassis_length = chassis_length
        self.chassis_width = chassis_width
        self.L = chassis_length / 2
        self.W = chassis_width / 2
        self.D = math.sqrt(self.L**2 + self.W**2)
    
    def twist_to_wheels(self, vx, vy, vz):
        """
        Convert Twist to wheel velocities
        
        Args:
            vx: Linear velocity X (m/s)
            vy: Linear velocity Y (m/s)
            vz: Angular velocity Z (rad/s)
        
        Returns:
            [v_fl, v_fr, v_bl, v_br] wheel velocities (rad/s)
        """
        v_fl = (vx - vy - vz * self.D) / self.wheel_radius
        v_fr = (vx + vy + vz * self.D) / self.wheel_radius
        v_bl = (vx + vy - vz * self.D) / self.wheel_radius
        v_br = (vx - vy + vz * self.D) / self.wheel_radius
        
        return [v_fl, v_fr, v_bl, v_br]
    
    def wheels_to_twist(self, v_fl, v_fr, v_bl, v_br):
        """Convert wheel velocities to Twist"""
        vx = self.wheel_radius * (v_fl + v_fr + v_bl + v_br) / 4.0
        vy = self.wheel_radius * (-v_fl + v_fr + v_bl - v_br) / 4.0
        vz = self.wheel_radius * (-v_fl + v_fr - v_bl + v_br) / (4.0 * self.D)
        
        return vx, vy, vz


class ArmKinematics:
    """Robotic arm kinematics calculations"""
    
    def __init__(self, upper_arm=0.15, forearm=0.15):
        """Initialize arm parameters"""
        self.L1 = upper_arm
        self.L2 = forearm
        self.max_reach = upper_arm + forearm
    
    def forward_kinematics(self, theta1, theta2):
        """Calculate end effector position from joint angles"""
        x = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        z = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)
        return x, z
    
    def inverse_kinematics(self, x, z):
        """
        Calculate joint angles from end effector position
        
        Returns:
            (theta1, theta2) or (None, None) if unreachable
        """
        r = math.sqrt(x**2 + z**2)
        
        if r > self.max_reach or r < abs(self.L1 - self.L2):
            return None, None
        
        cos_theta2 = (r**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        
        if abs(cos_theta2) > 1.0:
            return None, None
        
        theta2 = math.acos(cos_theta2)
        
        alpha = math.atan2(z, x)
        beta = math.atan2(
            self.L2 * math.sin(theta2),
            self.L1 + self.L2 * math.cos(theta2)
        )
        theta1 = alpha - beta
        
        return theta1, theta2


def create_twist(vx=0.0, vy=0.0, vz=0.0):
    """Create a Twist message"""
    twist = Twist()
    twist.linear = Vector3(x=vx, y=vy, z=0.0)
    twist.angular = Vector3(x=0.0, y=0.0, z=vz)
    return twist


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def clamp(value, min_val, max_val):
    """Clamp value between min and max"""
    return max(min_val, min(max_val, value))


def distance(p1, p2):
    """Calculate Euclidean distance between two points"""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def calculate_duration(distance_m, velocity_ms=0.5):
    """Calculate time to travel distance at given velocity"""
    if velocity_ms == 0:
        return 0
    return distance_m / velocity_ms
