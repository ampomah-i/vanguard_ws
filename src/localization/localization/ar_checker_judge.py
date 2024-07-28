#!/usr/bin/env python
from datetime import datetime
import rospy
import time
import threading
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
import datetime
import mavros
from mavros_msgs.msg import State
from std_msgs.msg import Int32

'''
A judging file for week 1 challenge.
Note that ar_checker_judged.lauch runs not this file, but the compiled .pyo file.
DON'T FORGET TO RECOMPILE AFTER CHANGES: python -O -m compileall ar_checker_judge.py
'''

#############
# CONSTANTS #
#############
MARKERS = [77, 76, 42, 74, 75, 40]
TARGET_DISTANCE  = 1.0
TARGET_THRESHOLD = 0.1
PERIOD = 1 # Message will print every _ seconds
RATE = 1 # hz
PENALTY = 1 # how many seconds each strike adds to final score

######################
# ARDISTCHECKERJUDGE #
######################
class ARDistCheckerJudge:

    def __init__(self):
        self.student_marker = None

        # A subscriber to the topic '/seen_tag'. calls its function when a specific tag number is recieved
        self.ar_tag_seen_sub = rospy.Subscriber("/seen_tag", Int32, self.student_tag_cb)
        
         # A subscriber to the topic '/ar_pose_marker'. calls its function when an ar tag is detected
        self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_cb)

        # A subscriber to the topic '/mavros/state'. self.update_state is called when a message of type 'State' is recieved
        self.state_subscriber = rospy.Subscriber("/mavros/state", State, self.update_state)

        # State of the drone. self.state.mode = flight mode, self.state.armed = are motors armed (True or False), etc.
        self.state = State()

        # List of all the "captured" markers
        self.captured = []
        # record of how many times the team detected an AR tag in the wrong order and sent it
        self.strikes = 0
        # The closest marker to the drone (in the z direction)
        self.marker = None
        # Points to the next marker in MARKERS that needs to be captured
        self.index = 0

        if len(MARKERS) > 0:
            # True if all markers have been captured, False otherwise
            self.done = False
        else:
            rospy.logerr("NO MARKERS GIVEN")
            self.done = True

    def student_tag_cb(self, tag_num):
        """
        sets the student_marker var to the tag last seen in range by the students and checks if they saw the right one.

        args:
            - tag_num : tag number last seen by student code, Int32
        """
        self.student_marker = tag_num.data
        self.check_dist()


    def update_state(self, state):
        """
        Callback function which is called when a new message of type State is recieved by self.state_subscriber

            Args:
                - state = mavros State message
        """
        self.state = state

    def ar_pose_cb(self,msg):
        """
        Given an AlvarMarkers message, sets self.current_marker to the closest observed marker (in the z direction).

            Args:
                - msg = AlvarMarkers message
        """
        #rospy.loginfo_throttle(PERIOD, 'Move to marker ' + str(MARKERS[self.index]))
        
        if len(msg.markers) > 0: 
            self.marker = min(msg.markers, key = lambda marker: marker.pose.pose.position.z) # closest tag
        else:
            self.marker = None
        

    def check_dist(self):
        """
        Determine whether the drone is within the goal distance of the marker and print the appropriate message the log.

            - The drone has already "seen" the marker: "Marker <marker.id> has already been seen"
        TODO:
         - needs to check we actually saw a tag
         - needs to check we
        """
        
        if self.done:
            rospy.loginfo("Already done! You saw everything!")
            return
            
        target_marker = MARKERS[self.index]
        
        if self.marker != None:
            # Check if the closest marker is next marker that needs to be captured
            if self.student_marker == target_marker and self.marker.id == target_marker:
                # Drone is within the threshold
                if abs(self.marker.pose.pose.position.z - TARGET_DISTANCE) < TARGET_THRESHOLD:
                    rospy.loginfo("CAPTURED " + str(self.student_marker))
                    self.captured.append(self.student_marker)
                    self.index += 1
                    
                else:
                    rospy.loginfo("Strike. Not in range to see that marker. Move " + str(round(self.marker.pose.pose.position.z - TARGET_DISTANCE, 2)) + " meters to capture " + str(self.marker.id))
                    self.strikes += 1
            
            else:
                rospy.loginfo("Strike. You are looking for " + str(target_marker) + " but you saw " + str(self.marker.id))
                self.strikes += 1
        else:
            rospy.loginfo("Strike. No marker is visible, but you saw " + str(self.student_marker))
            self.strikes += 1
                
        self.check_end()

    def check_end(self):
        """
        Determine whether the challenge is finished, AKA if all the tags have been detected correctly, and then calculates score
        """
        # Check if challenge has been completed
        if len(self.captured) == len(MARKERS):
            timedelta = time.time() - self.start_time
            score = timedelta + (self.strikes * PENALTY)
            rospy.loginfo("DONE!")
            rospy.loginfo("Your time was: " + str(timedelta))
            rospy.loginfo("You had " + str(self.strikes) + " strike(s), so final score is: " + str(score))
            self.done = True
        else:
            self.done = False


if __name__ == '__main__':

    rospy.init_node('ar_checker_judge')
    a = ARDistCheckerJudge()

    while not rospy.is_shutdown() and a.state.mode != 'POSCTL':
        
        print('Waiting to enter POSCTL mode')
        
        # Publish at the desired rate
        rospy.Rate(RATE).sleep()

    a.start_time = time.time() # time at start of current period
    # List of all the "captured" markers
    a.captured = []
    # record of how many times the team detected an AR tag in the wrong order and sent it
    a.strikes = 0
    # The closest marker to the drone (in the z direction)
    a.marker = None
    # Points to the next marker in MARKERS that needs to be captured
    a.index = 0

    rospy.spin()