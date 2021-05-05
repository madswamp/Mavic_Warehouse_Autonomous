#!/usr/bin/env python2
import math


import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs


class MavicFreebotsMission:

    def __init__(self):
        self.drone0pose_sub = rospy.Subscriber('/freebots/drone_pose', PoseStamped, self.drone0posecallback)
        self.drone0speed_sub = rospy.Subscriber('/drone0/raw_localization/linear_speed', TwistStamped, self.drone0speedcallback)
        self.robotstate_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.robotstatecallback)
        self.missioncommand_sub = rospy.Subscriber('/freebots/mission', String, self.missioncommandcallback)
        self.missiontags_sub = rospy.Subscriber('/freebots/tags', PoseStamped, self.missiontagscallback)

        self.waypoint_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # debug
        self.mission_state_pub = rospy.Publisher('/robot/Mission_State', String, queue_size=10)
        self.goal_pub = rospy.Publisher('/robot/freebots_goal', PoseStamped, queue_size=10)

        # # tf2
        #
        # self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # tf buffer length
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.MissionState = 0

        self.mission_rate = rospy.Rate(10)

        self.mission_following_waypoint = False

        self.tag_order = ["tag_4", "tag_19", "tag_3", "tag_5", "tag_8", "tag_9", "victim"]
        self.tag_index = 0
        self.tags = {}

        self.mission_start = False

        self.drone_x = None
        self.drone_y = None
        self.drone_vx = None
        self.drone_vy = None

    def drone0posecallback(self, msg):

        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y

    def drone0speedcallback(self, msg):

        self.drone_vx = msg.twist.linear.x
        self.drone_vy = msg.twist.linear.y

    def robotstatecallback(self, msg):
        if self.mission_following_waypoint:
            for status in msg.status_list:
                if status.status == 1:
                    return
            self.mission_following_waypoint = False

    def missioncommandcallback(self, msg):
        if msg.data == "Move":
            self.mission_start = True

    def missiontagscallback(self, msg):
        if msg.header.frame_id not in self.tags:
            self.tags[msg.header.frame_id] = msg.pose
        else:
            self.tags[msg.header.frame_id] = msg.pose

    def checkDronePositionTag(self, tag):
        if abs(self.tags[tag].position.x - self.drone_x) < 0.5 and abs(self.tags[tag].position.y - self.drone_y) < 0.5 and self.drone_vx < 0.01 and self.drone_vy < 0.01:
            return True
        else:
            return False

    def sendWaypoint(self):

        # ter cuidado com o tamanho do index
        if self.tag_order[self.tag_index] == "victim":
            waypointstamped = self.calculateWaypoint(self.tag_order[self.tag_index], self.tag_order[self.tag_index-1], self.tag_order[self.tag_index])
        else:
            waypointstamped = self.calculateWaypoint(self.tag_order[self.tag_index], self.tag_order[self.tag_index], self.tag_order[self.tag_index+1])

        self.waypoint_pub.publish(waypointstamped)
        self.goal_pub.publish(waypointstamped)

    def calculateWaypoint(self, tag1, tag2, tag3):

        waypoint = PoseStamped()

        yaw_to_goal = math.atan2(self.tags[tag3].position.y - self.tags[tag2].position.y,
                                 self.tags[tag3].position.x - self.tags[tag2].position.x)

        quaternion = quaternion_from_euler(0, 0, yaw_to_goal)

        waypoint.pose.orientation.x = quaternion[0]
        waypoint.pose.orientation.y = quaternion[1]
        waypoint.pose.orientation.z = quaternion[2]
        waypoint.pose.orientation.w = quaternion[3]

        # if tag1 == tag3:
        #     dist = math.sqrt(pow(self.tags[tag1].position.x - self.tags[tag2].position.x,2)+pow(self.tags[tag1].position.y - self.tags[tag2].position.y,2))
        #     new_dist = dist - 0.5
        #     waypoint.pose.position.x = new_dist * math.cos(yaw_to_goal)
        #     waypoint.pose.position.y = new_dist * math.sin(yaw_to_goal)
        # else:
        #     waypoint.pose.position.x = self.tags[tag1].position.x
        #     waypoint.pose.position.y = self.tags[tag1].position.y

        dist = math.sqrt(pow(self.tags[tag1].position.x - self.tags[tag2].position.x,2)+pow(self.tags[tag1].position.y - self.tags[tag2].position.y,2))
        new_dist = dist - 0.5
        waypoint.pose.position.x = self.tags[tag1].position.x + new_dist * math.cos(yaw_to_goal)
        waypoint.pose.position.y = self.tags[tag1].position.y + new_dist * math.sin(yaw_to_goal)
        

        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = "map"

        return waypoint

    def mission(self):
        mission_state_msg = String()
        print("Starting Mission")
        while not rospy.is_shutdown():
            mission_state_msg.data = str(self.MissionState)
            self.mission_state_pub.publish(mission_state_msg)
            if self.MissionState == 0:
                if self.mission_start == True:
                    self.MissionState = 1

            elif self.MissionState == 1:
                # Tag 4
                if self.checkDronePositionTag(self.tag_order[self.tag_index]):
                    self.mission_following_waypoint = True
                    self.MissionState = 2
                    self.sendWaypoint()

            elif self.MissionState == 2:

                if self.mission_following_waypoint == False:
                    self.tag_index = self.tag_index + 1
                    self.MissionState = 3

            elif self.MissionState == 3:
                # Tag 19
                if self.checkDronePositionTag(self.tag_order[self.tag_index]):
                    self.mission_following_waypoint = True
                    self.MissionState = 4
                    self.sendWaypoint()

            elif self.MissionState == 4:

                if self.mission_following_waypoint == False:
                    self.tag_index = self.tag_index + 1
                    self.MissionState = 5

            elif self.MissionState == 5:
                # Tag 3
                if self.checkDronePositionTag(self.tag_order[self.tag_index]):
                    self.mission_following_waypoint = True
                    self.MissionState = 6
                    self.sendWaypoint()

            elif self.MissionState == 6:

                if self.mission_following_waypoint == False:
                    self.tag_index = self.tag_index + 1
                    self.MissionState = 7

            elif self.MissionState == 7:
                # Tag 5
                if self.checkDronePositionTag(self.tag_order[self.tag_index]):
                    self.mission_following_waypoint = True
                    self.MissionState = 8
                    self.sendWaypoint()

            elif self.MissionState == 8:

                if self.mission_following_waypoint == False:
                    self.tag_index = self.tag_index + 1
                    self.MissionState = 9

            elif self.MissionState == 9:
                # Tag 8
                if self.checkDronePositionTag(self.tag_order[self.tag_index]):
                    self.mission_following_waypoint = True
                    self.MissionState = 10
                    self.sendWaypoint()

            elif self.MissionState == 10:

                if self.mission_following_waypoint == False:
                    self.tag_index = self.tag_index + 1
                    self.MissionState = 11

            elif self.MissionState == 11:
                # Tag 9
                if self.checkDronePositionTag(self.tag_order[self.tag_index]):
                    self.mission_following_waypoint = True
                    self.MissionState = 12
                    self.sendWaypoint()

            elif self.MissionState == 12:

                if self.mission_following_waypoint == False:
                    self.tag_index = self.tag_index + 1
                    self.MissionState = 13

            elif self.MissionState == 13:
                # Tag Victim
                if self.checkDronePositionTag(self.tag_order[self.tag_index]):
                    self.mission_following_waypoint = True
                    self.MissionState = 14
                    self.sendWaypoint()

            elif self.MissionState == 14:

                if self.mission_following_waypoint == False:
                    self.tag_index = self.tag_index + 1
                    self.MissionState = 15

            elif self.MissionState == 15:
                print("EZZZ")
                self.MissionState = 16


            self.mission_rate.sleep()


if __name__ == '__main__':
    rospy.init_node('freebots_mission_node')
    mission = MavicFreebotsMission()
    mission.mission()
