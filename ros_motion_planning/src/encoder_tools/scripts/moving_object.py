#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
moving_object.py

Move Gazebo models along waypoints defined in pedestrian config YAML.
Reads entries with type: "box" and publishes set_model_state to Gazebo.

Usage:
    rosrun encoder_tools moving_object.py _config:=/path/to/pedestrian_config.yaml
"""

import rospy
import yaml
import math
import time
from threading import Thread
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist


class MovingObject:
    def __init__(self, config_path):
        with open(config_path, 'r') as f:
            self.cfg = yaml.safe_load(f)

        self.objects = []
        for obj in self.cfg.get("pedestrians", {}).get("ped_property", []):
            if obj.get("type") == "box":
                self.objects.append(obj)
                rospy.loginfo("MovingObject: found '%s'", obj["name"])

        if not self.objects:
            rospy.logwarn("MovingObject: no box-type objects found in config")

        # Wait for Gazebo services
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.running = True
        self.threads = []

    def _parse_pose(self, pose_str):
        """Parse 'x y z roll pitch yaw' string to Pose"""
        parts = list(map(float, pose_str.strip().split()))
        p = Pose()
        p.position.x = parts[0]
        p.position.y = parts[1]
        p.position.z = parts[2]
        return p

    def _interpolate(self, p1, p2, t):
        """Linear interpolation between two poses"""
        p = Pose()
        p.position.x = p1.position.x + t * (p2.position.x - p1.position.x)
        p.position.y = p1.position.y + t * (p2.position.y - p1.position.y)
        p.position.z = p1.position.z + t * (p2.position.z - p1.position.z)
        return p

    def _move_object(self, obj):
        """Move a single box along its waypoints"""
        name = obj["name"]
        velocity = obj.get("velocity", 1.0)
        cycle = obj.get("cycle", False)
        update_rate = self.cfg.get("pedestrians", {}).get("update_rate", 10)

        # Parse waypoints
        waypoints = []
        for key in sorted(obj["trajectory"].keys()):
            waypoints.append(self._parse_pose(obj["trajectory"][key]))

        if len(waypoints) < 2:
            rospy.logerr("%s: need at least 2 waypoints", name)
            return

        rate = rospy.Rate(update_rate)

        while self.running and not rospy.is_shutdown():
            for wp_idx in range(len(waypoints) - 1):
                p_start = waypoints[wp_idx]
                p_end = waypoints[wp_idx + 1]

                # Distance between waypoints
                dx = p_end.position.x - p_start.position.x
                dy = p_end.position.y - p_start.position.y
                dz = p_end.position.z - p_start.position.z
                dist = math.hypot(dx, math.hypot(dy, dz))

                if dist < 0.01:
                    continue

                duration = dist / velocity
                steps = max(int(duration * update_rate), 1)

                for step in range(steps):
                    if not self.running or rospy.is_shutdown():
                        return

                    t = (step + 1) / steps
                    pose = self._interpolate(p_start, p_end, t)

                    state = ModelState()
                    state.model_name = name
                    state.pose = pose
                    state.reference_frame = "world"

                    try:
                        self.set_state(state)
                    except rospy.ServiceException as e:
                        rospy.logwarn("%s: set_model_state failed: %s", name, e)
                        return

                    rate.sleep()

            if not cycle:
                rospy.loginfo("%s: reached final waypoint, done", name)
                return

    def start(self):
        """Start all object threads"""
        for obj in self.objects:
            t = Thread(target=self._move_object, args=(obj,), daemon=True)
            t.start()
            self.threads.append(t)
            rospy.loginfo("MovingObject: started '%s'", obj["name"])

    def stop(self):
        self.running = False
        for t in self.threads:
            t.join(timeout=1.0)


if __name__ == "__main__":
    rospy.init_node("moving_object")
    config_path = rospy.get_param("~config", "")

    if not config_path:
        # Default: same directory as user_config
        import os
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, "../../../user_config/pedestrian_config_four.yaml")
        config_path = os.path.abspath(config_path)

    rospy.loginfo("MovingObject: loading config from %s", config_path)

    if not os.path.exists(config_path):
        rospy.logfatal("Config not found: %s", config_path)
        exit(1)

    mover = MovingObject(config_path)

    if mover.objects:
        mover.start()
        rospy.spin()
    else:
        rospy.logwarn("MovingObject: no boxes to move, exiting")

    mover.stop()
