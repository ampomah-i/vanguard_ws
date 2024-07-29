#!/usr/bin/env python
from datetime import datetime
import rospy
import time
import threading
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Int32


TARGET_DIS    = 1.0  # target distance away that we want to see the tag
TARGET_HEIGHT    = 0.6  # height the student should fly at
HEIGHT_ERROR    = .1 #allowable flight height error 

'''
You need to fly the drone in position control mode to see the AR tag
'''

class BasicAvoider:
    def __init__(self):
        '''
        Initializes class: creates subscriber to detect AR tags and height
        
        AR tag data is published on "/ar_pose_marker" with a message of type AlvarMarkers
        
        '''
        rospy.loginfo("BasicAvoider Started!")

        self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_cb)

        # A subscriber to the topic '/mavros/local_position/pose. self.pos_sub_cb is called when a message of type 'PoseStamped'
        # is recieved 
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pos_sub_cb)

    ######################
    # CALLBACK FUNCTIONS #
    ######################

    def ar_pose_cb(self,msg):
        '''Callback for when the drone sees an AR tag
        Parameters
        ----------
        msg : ar_pose_marker
            list of the poses of all the observed AR tags, with respect to the output frame
        '''
        if msg.markers  == []:
            return

        marker = min(msg.markers, key=lambda p: p.pose.pose.position.z)
        self.current_marker = marker
        self.check_dist()

    def pos_sub_cb(self, posestamped):
        '''
        checks if flying in correct height zone
        '''
        self.height = posestamped.pose.position.z
        if abs(self.height-TARGET_HEIGHT)<=HEIGHT_ERROR:
            rospy.loginfo("Good job! You are flying at "+str(self.height)+" which is in the green zone of the target height: " + str(TARGET_HEIGHT))
        else:
            rospy.loginfo("CAUTION You are flying at "+str(self.height)+" which is not in the green zone of the target height: " + str(TARGET_HEIGHT))

###OTHER FUNCTIONS###

    def check_dist(self):
        '''
        Finds height of nearest AR tag if it is close enough
        '''
        marker = self.current_marker
        
        if (marker.pose.pose.position.z <= TARGET_DIS):
            # new tag + close enough, add to seen

            # Calculate the height of the obstical above the ground
            obstical_height = self.height - marker.pose.pose.position.y
            self.command_flyer(obstical_height)

        else:
            # marker isn't close enough yet, but we see one
            rospy.loginfo("Not there yet: move " + str(marker.pose.pose.position.z - TARGET_DIS) + " meters to capture " + str(marker.id))

    def command_flyer(self, ar_height):
        '''
        tell the user to go uo or down depending on the ar height
        '''
        if ar_height < TARGET_HEIGHT:
            rospy.loginfo("Go OVER marker")
        else:
            rospy.loginfo("Go UNDER marker")



if __name__ == '__main__':
    rospy.init_node('final_challenge_sol')
    ba = BasicAvoider()

    rospy.spin()
