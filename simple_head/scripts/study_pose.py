#!/usr/bin/env python

import actionlib
import rospy
import socket
import thread
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from dynamixel_controllers.srv import SetSpeed, SetTorqueLimit
from trajectory_msgs.msg import JointTrajectoryPoint
from simple_head.msg import PoseCommand


class PoseGoer:
    CONFIG_SRV_TYPES = {"speed": SetSpeed, "torque_limit": SetTorqueLimit}

    def __init__(self):

        # Initialize Node
        rospy.init_node("goer")

        # Poses and DoFs from param file
        self.poses = {}
        self.dofs = []
        self.load_poses()

        # Check if user is distracted
        self.distracted = False
        self.socket = socket.socket()

        # Connect to Action Server
        self.ac = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.ac.wait_for_server()
        rospy.loginfo('Connected to joint trajectory action!')

        # Set (potentially higher) Motor Speed and Torque Limit from param file
        self.set_default_configs()

        # Subscribe to the "/goto_pose" topic and wait for PoseCommand msgs
        rospy.Subscriber("/goto_pose", PoseCommand, self.pose_request)

    def load_poses(self):
        """
        Converts each pose from the YAML file into a goal and creates members poses and dofs:
        poses is a dictionary mapping name (e.g. down_right) -> goal.
        
        """
        param_poses = rospy.get_param("~poses")
        for name, parameters in param_poses.iteritems():
            goal = FollowJointTrajectoryGoal()
            # self.dofs is the list of joint names (i.e. head_roll, head_tilt, neck_pan, neck_tilt)
            self.dofs = parameters.keys()

            point = JointTrajectoryPoint()
            point.positions = parameters.values()

            goal.trajectory.joint_names = self.dofs
            goal.trajectory.points.append(point)

            self.poses[name] = goal

    def set_config(self, param, value):
        """
        Set a configuration parameter in a set of DoF controllers

        @param param: the parameter (e.g. pose_speed, pose_torque_limit) to set
        @param value: value to set the parameter to
        """
        service_names = ['/' + dof + '_controller/set_' + param for dof in self.dofs]
        for sn in service_names:
            rospy.wait_for_service(sn)
            service = rospy.ServiceProxy(sn, PoseGoer.CONFIG_SRV_TYPES[param])
            service.call(value)

    def set_default_configs(self):
        """
        Get values from the parameter server and set the configuration in the controller
        """
        for param in PoseGoer.CONFIG_SRV_TYPES.keys():
            value = rospy.get_param("~pose_" + param)
            self.set_config(param, value)


    def pose_request(self, msg):
        """
        Callback for topic subscriber. Basically extracts the data from the
        msg and sends it to goto_pose()
        
        @param msg: ROS message data
        """
        print "Requested pose %s" % msg.pose
        self.goto_pose(msg.pose, msg.duration)

    def goto_pose(self, pose, duration):
        """
        Go to pose by publishing command to each controller topic

        @param pose: pose dictionary key in param server
        @param duration: duration for trajectory
        """
        if pose in self.poses:
            goal = self.poses[pose]
            goal.trajectory.header.stamp = rospy.Time.now()
            goal.trajectory.points[0].time_from_start = duration  # Assuming only one point per trajectory
            self.ac.send_goal(goal)
        else:
            rospy.logerr("No such pose: %s" % pose)

    def face_user(self):
        self.goto_pose('face_user', rospy.Duration.from_sec(3))
        rospy.sleep(3)

    def show_agreement(self):
        self.goto_pose('face_user_closer', rospy.Duration.from_sec(3))
        rospy.sleep(3)
        self.goto_pose('face_user_nod_head_down', rospy.Duration.from_sec(0.75))
        rospy.sleep(1)
        self.goto_pose('face_user_nod_head_up', rospy.Duration.from_sec(0.75))
        rospy.sleep(1)

    def show_disagreement(self):
        self.goto_pose('face_user_farther_solemn', rospy.Duration.from_sec(3))
        rospy.sleep(3)

        # Shake head for 3 times
        for i in range(3):
            self.goto_pose('face_user_shake_head_left', rospy.Duration.from_sec(0.5))
            rospy.sleep(0.5)
            self.goto_pose('face_user_shake_head_right', rospy.Duration.from_sec(1))
            rospy.sleep(1.0)
            self.goto_pose('face_user_shake_head_middle', rospy.Duration.from_sec(0.5))
            rospy.sleep(0.5)


    def watch_screen(self):
        self.goto_pose('watch_screen', rospy.Duration.from_sec(4))
        rospy.sleep(4)
        self.goto_pose('watch_screen_closer', rospy.Duration.from_sec(2))
        rospy.sleep(3)
        self.goto_pose('watch_screen_nod_head_down', rospy.Duration.from_sec(0.75))
        rospy.sleep(1)
        self.goto_pose('watch_screen_nod_head_up', rospy.Duration.from_sec(0.75))
        rospy.sleep(1)

    # goto_distracted_pose(self):

    def connect_to_server(self):
        ip = '127.0.0.1'
        port = 20000
        self.socket.connect((ip, port))
        print 'Connection with the prediction service is now established'
        while True:
            result = goer.socket.recv(1024)
            if result == "Distracted":
                self.distracted = True


    



if __name__ == '__main__':
    try:
        goer = PoseGoer()
        goer.face_user()

        # Start a new thread to listen for the user state from the prediction service
        thread.start_new_thread(goer.connect_to_server, ())
        
        
        while not rospy.is_shutdown():
            # Go to the initial position (watch at the screen)
            if not goer.distracted:
                goer.watch_screen()
                rospy.sleep(2)
                goer.show_agreement()
                goer.face_user()
            else:
                goer.show_disagreement()
                goer.face_user()
            
            break            
        



        '''
        goer.goto_face_user_pose()
        rospy.sleep(1)
        goer.goto_show_agreement_pose()
        goer.goto_face_user_pose()
        rospy.sleep(1)
        goer.wa()
        goer.goto_face_user_pose()
        rospy.sleep(1)
        #goer.goto_study_pose()
        '''
    except rospy.ROSInterruptException:
        rospy.loginfo("Bye bye...")
