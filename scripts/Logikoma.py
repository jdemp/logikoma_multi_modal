#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from logikoma_multi_modal.msg import action_output
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from tf import transformations
import math
from random import randint

def euler_to_quat(roll,pitch,yaw):
    return transformations.quaternion_from_euler(roll,pitch,yaw)


def quat_to_euler(quat):
    return transformations.euler_from_quaternion(quat[0], quat[1], quat[2], quat[3])


def rotate_around_z(degrees, quat):
    euler = quat_to_euler(quat)
    return euler_to_quat(euler[0],euler[1],math.radians(degrees))


#need to transform coordinates from odom frame to map frame before sending the action to the robot
#look at sending the action to the /goal topic instead of the action client
class Logikoma:
    def __init__(self):
        self.goal_topic = '/user_goal'
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.pose = Pose()
        self.stopped = True
        self.auto = False
        self.local_map = {'FL':0,'F':0,'FR':0,'L':0,'R':0,'BL':0,'B':0,'BR':0}
        self.previous_move = 'stop'
        self.pref_moves = []
        self.secondary_moves = []
        self.stuck = False

    def go_forward(self, action):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 1 #3 meters
        goal.target_pose.pose.orientation.w = 1.0 #go forward
        self.move_base.send_goal(goal)

    def navigate_to_point(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = self.pose.position.z
        goal.target_pose.pose.orientation = self.pose.orientation
        self.move_base.send_goal(goal)

    def go_back(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = -1
        goal.target_pose.pose.orientation.z = 1
        self.move_base.send_goal(goal)

    def turn(self, left):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = .5
        if left:
            goal.target_pose.pose.position.y = .5
            goal.target_pose.pose.orientation.z = .7071
            goal.target_pose.pose.orientation.w = .7071
        else:
            goal.target_pose.pose.position.y = -.5
            goal.target_pose.pose.orientation.z = .7071
            goal.target_pose.pose.orientation.w = .7071
        self.move_base.send_goal(goal)

    def hard_turn(self,left):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        if left:
            goal.target_pose.pose.position.y = 1
            goal.target_pose.pose.orientation.z = .7071
            goal.target_pose.pose.orientation.w = .7071
        else:
            goal.target_pose.pose.position.y = -1
            goal.target_pose.pose.orientation.z = .7071
            goal.target_pose.pose.orientation.w = .7071
        self.move_base.send_goal(goal)

    def rotate(self, direction, degree):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        if direction == 'right':
            goal.target_pose.pose.orientation.z = .7071
            goal.target_pose.pose.orientation.w = .7071
        elif direction == 'left':
            goal.target_pose.pose.orientation.z = .7071
            goal.target_pose.pose.orientation.w = .7071
        else:
            goal.target_pose.pose.orientation.w = -1
        self.move_base.send_goal(goal)

    def done(self):
        self.stopped = True

    def process(self,user_goal):
        # create list of goals and which method to call each method will parse the action
        #valid_action = True
        if user_goal.action =='stop':
            self.move_base.cancel_goal()
            self.stopped = True
        elif user_goal.action == 'continue':
            pass
        elif user_goal.action == 'search':
            self.stopped = False
        elif user_goal.action == 'done':
            self.done()
        elif user_goal.action == 'go back':
            #self.move_base.cancel_goal()
            #self.go_back()
            self.local_map['B'] = 1
        elif user_goal.action == 'turn left':
            #self.move_base.cancel_goal()
            #self.turn('left')
            self.local_map['FL'] = 1
            self.local_map['F'] =.5
            self.local_map['L'] =1
        elif user_goal.action == 'turn right':
            #self.move_base.cancel_goal()
            #self.turn('right')
            self.local_map['FR'] = 1
            self.local_map['F'] =.5
            self.local_map['R'] =1
        elif user_goal.action == 'hard right':
            #self.move_base.cancel_goal()
            #self.turn('right')
            self.local_map['FR'] = .5
            self.local_map['F'] =.25
            self.local_map['R'] =1
        elif user_goal.action == 'hard left':
            #self.move_base.cancel_goal()
            #self.turn('right')
            self.local_map['FL'] = .5
            self.local_map['F'] =.25
            self.local_map['L'] =1
        elif user_goal.action == 'go straight':
            #self.move_base.cancel_goal()
            #self.go_forward(1)
            self.local_map['F'] = 1
        elif user_goal.action == 'obstacle right':
            self.local_map['BR'] = -.5
            self.local_map['FR'] =-.5
            self.local_map['R'] = -1
        elif user_goal.action == 'obstacle left':
            self.local_map['BL'] = -.5
            self.local_map['FL'] =-.5
            self.local_map['L'] = -1
        elif user_goal.action == 'obstacle front':
            self.local_map['FL'] = -.5
            self.local_map['FR'] =-.5
            self.local_map['F'] = -1
        elif user_goal.action.split(':')[0] == 'rotate':
            #self.move_base.cancel_goal()
            #self.rotate(user_goal.action.split(':')[1], user_goal.action.split(':')[2])
            pass
        else:
            print "Not useful information"

    def update_pose(self,msg):
        self.pose = msg.pose.pose

    def on_shutdown(self):
        #save action history to file
        print "Ending"

    def reset_local_map(self):
        self.local_map = {'FL':0,'F':0,'FR':0,'L':0,'R':0,'BL':0,'B':0,'BR':0}

    def autonomous(self):
        self.pref_moves = []
        self.secondary_moves  = []
        for dir in self.local_map.keys():
            if self.local_map[dir] > 0:
                potential_move = (dir,self.local_map[dir])
                self.pref_moves.append(potential_move)
            elif self.local_map[dir] == 0:
                self.secondary_moves.append(dir)

        if len(self.pref_moves) == 0:
            print "Moving with no Prefs"
            print str(self.secondary_moves)
            if self.stuck:
                self.go_back()
                self.stopped = True
                self.stuck = False
            elif 'F' in self.secondary_moves:
                print "Moving forward"
                self.go_forward(1)
                success = self.move_base.wait_for_result(rospy.Duration(20))
                if not success:
                    self.reset_local_map()
                    self.local_map['FL']=-5
                    self.local_map['F']=-1
                    self.local_map['FR']=-.5
                else:
                    self.reset_local_map()
            elif 'L' in self.secondary_moves and 'R' in self.secondary_moves:
                print "Turning"
                selection = randint(0,1)
                if selection ==0:
                    self.hard_turn(True)
                else:
                    self.hard_turn(False)
                success = self.move_base.wait_for_result(rospy.Duration(20))
                if not success:
                    print "I am stuck please help couldn't turn"
                    self.stuck = True
                    self.stopped = True
                    #self.auto = False
                else:
                    self.reset_local_map()
            elif 'L' in self.secondary_moves:
                print "Turning left"
                self.hard_turn(True)
                success = self.move_base.wait_for_result(rospy.Duration(20))
                if not success:
                    print "I am stuck please help couldn't turn left"
                    self.stopped = True
                    #self.auto = False
                else:
                    self.reset_local_map()
            elif 'R' in self.secondary_moves:
                print "turning right"
                self.hard_turn(False)
                success = self.move_base.wait_for_result(rospy.Duration(20))
                if not success:
                    print "I am stuck please help couldn't turn right"
                    self.stopped = True
                    #self.auto = False
                else:
                    self.reset_local_map()
            else:
                print "Going back"
                self.go_back()
                success = self.move_base.wait_for_result(rospy.Duration(20))
                self.stopped = True
                self.reset_local_map()
        else:
            print "Moving with Prefs"
            best_move = ('none',0)
            for option in self.pref_moves:
                if option[1]>best_move[1]:
                    best_move = option
                elif option[1]==best_move[1]:
                    selection = randint(0,1)
                    if selection ==0:
                        best_move = option
            if best_move[0] == 'F':
                self.go_forward(1)
            elif best_move[0] == 'FL':
                self.turn(True)
            elif best_move[0] == 'FR':
                self.turn(False)
            elif best_move[0] == 'L':
                self.hard_turn(True)
            elif best_move[0] == 'R':
                self.hard_turn(False)
            else:
                self.go_back()
            success = self.move_base.wait_for_result(rospy.Duration(20))
            self.reset_local_map()
            if not success:
                print "Something went wrong, please help"
                self.stopped = True
            else:
                print "I moved to" + best_move[0]


    def run(self):
        rospy.Subscriber(self.goal_topic, action_output, callback=self.process)
        rospy.Subscriber('/odom', Odometry, callback=self.update_pose)
        rospy.on_shutdown(self.on_shutdown)
        rate = rospy.Rate(.1)
        while not rospy.is_shutdown():
            if not self.stopped:
                self.autonomous()
            else:
                print "I am stopped please help me"
            rate.sleep()




if __name__ == '__main__':
    rospy.init_node('Logikoma')
    n = Logikoma()
    n.run()