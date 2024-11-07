# coding=utf8
import time
import copy
import socket
import struct
import numpy as np
import math
from real.robotiq_gripper import RobotiqGripper
from real.realsenseD415 import Camera


class UR_Robot:
    def __init__(self, tcp_host_ip="192.168.50.100", tcp_port=30003, workspace_limits=None,
                 is_use_robotiq85=True, is_use_camera=True):
        # Init varibles
        if workspace_limits is None:
            workspace_limits = [[-0.7, 0.7], [-0.7, 0.7], [0.00, 0.6]]
        self.workspace_limits = workspace_limits
        self.tcp_host_ip = tcp_host_ip
        self.tcp_port = tcp_port
        self.is_use_robotiq85 = is_use_robotiq85
        self.is_use_camera = is_use_camera

        # UR5 robot configuration
        # Default joint/tool speed configuration
        self.joint_acc = 1.4  # Safe: 1.4   8
        self.joint_vel = 1.05  # Safe: 1.05  3

        # Joint tolerance for blocking calls
        self.joint_tolerance = 0.01

        # Default tool speed configuration
        self.tool_acc = 0.5  # Safe: 0.5
        self.tool_vel = 0.2  # Safe: 0.2

        # Tool pose tolerance for blocking calls
        self.tool_pose_tolerance = [0.002, 0.002, 0.002, 0.01, 0.01, 0.01]

        # robotiq85 gripper configuration
        if (self.is_use_robotiq85):
            # reference https://gitlab.com/sdurobotics/ur_rtde
            # Gripper activate
            self.gripper = RobotiqGripper()
            # don't change the 63352 port
            self.gripper.connect(self.tcp_host_ip, 63352)
            self.gripper._reset()
            print("Activating gripper...")
            self.gripper.activate()
            time.sleep(1.5)

        # realsense configuration
        if (self.is_use_camera):
            # Fetch RGB-D data from RealSense camera
            self.camera = Camera()
            # self.cam_intrinsics = self.camera.intrinsics  # get camera intrinsics
        self.cam_intrinsics = np.array(
            [615.284, 0, 309.623, 0, 614.557, 247.967, 0, 0, 1]).reshape(3, 3)
        # # Load camera pose (from running calibrate.py), intrinsics and depth scale
        self.cam_pose = np.loadtxt(
            'real/cam_pose/camera_pose.txt', delimiter=' ')
        self.cam_depth_scale = np.loadtxt(
            'real/cam_pose/camera_depth_scale.txt', delimiter=' ')

        # Default robot home joint configuration (the robot is up to air)
        self.home_joint_config = [-(0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
                                  (0 / 360.0) * 2 * np.pi, -
                                  (90 / 360.0) * 2 * np.pi,
                                  -(0 / 360.0) * 2 * np.pi, 0.0]

        # test
        # self.testRobot()
    # Test for robot controlmove_and_wait_for_pos
    def testRobot(self):
        try:
            print("Test for robot...")
            # self.move_j([-(0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
            #                  (0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
            #                  -(0 / 360.0) * 2 * np.pi, 0.0])
            # self.move_j([(57.04 / 360.0) * 2 * np.pi, (-65.26/ 360.0) * 2 * np.pi,
            #                  (73.52/ 360.0) * 2 * np.pi, (-100.89/ 360.0) * 2 * np.pi,
            #                  (-86.93/ 360.0) * 2 * np.pi, (-0.29/360)*2*np.pi])
            # self.open_gripper()
            # self.move_j([(57.03 / 360.0) * 2 * np.pi, (-56.67 / 360.0) * 2 * np.pi,
            #                   (88.72 / 360.0) * 2 * np.pi, (-124.68 / 360.0) * 2 * np.pi,
            #                   (-86.96/ 360.0) * 2 * np.pi, (-0.3/ 360) * 2 * np.pi])
            # self.close_gripper()
            # self.move_j([(57.04 / 360.0) * 2 * np.pi, (-65.26 / 360.0) * 2 * np.pi,
            #                   (73.52 / 360.0) * 2 * np.pi, (-100.89 / 360.0) * 2 * np.pi,
            #                   (-86.93 / 360.0) * 2 * np.pi, (-0.29 / 360) * 2 * np.pi])
            # self.move_j([-(0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
            #                  (0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
            #                  -(0 / 360.0) * 2 * np.pi, 0.0])
            # self.move_j_p([0.3,0,0.3,np.pi/2,0,0],0.5,0.5)
            # for i in range(10):
            #     self.move_j_p([0.3, 0, 0.3, np.pi, 0, i*0.1], 0.5, 0.5)
            #     time.sleep(1)
            # self.move_j_p([0.3, 0, 0.3, -np.pi, 0, 0],0.5,0.5)
            # self.move_p([0.3, 0.3, 0.3, -np.pi, 0, 0],0.5,0.5)
            # self.move_l([0.2, 0.2, 0.3, -np.pi, 0, 0],0.5,0.5)
            # self.plane_grasp([0.3, 0.3, 0.1])
            # self.plane_push([0.3, 0.3, 0.1])
        except:
            print("Test fail! ")

    # joint control
    ''' 
    input:joint_configuration = joint angle 
    '''

    def move_j(self, joint_configuration, k_acc=1, k_vel=1, t=0, r=0):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        # "movej([]),a=,v=,\n"
        tcp_command = "movej([%f" % joint_configuration[0]
        for joint_idx in range(1, 6):
            tcp_command = tcp_command + \
                (",%f" % joint_configuration[joint_idx])
        tcp_command = tcp_command + \
            "],a=%f,v=%f,t=%f,r=%f)\n" % (
                k_acc*self.joint_acc, k_vel*self.joint_vel, t, r)
        self.tcp_socket.send(str.encode(tcp_command))

        # Block until robot reaches home state
        state_data = self.tcp_socket.recv(1500)
        actual_joint_positions = self.parse_tcp_state_data(
            state_data, 'joint_data')
        while not all([np.abs(actual_joint_positions[j] - joint_configuration[j]) < self.joint_tolerance for j in range(6)]):
            state_data = self.tcp_socket.recv(1500)
            actual_joint_positions = self.parse_tcp_state_data(
                state_data, 'joint_data')
            time.sleep(0.01)
        self.tcp_socket.close()
    # joint control
    ''' 
    move_j_p(self, tool_configuration,k_acc=1,k_vel=1,t=0,r=0) 
    input:tool_configuration=[x y z r p y] 
   這個方法是基於末端執行器位置的運動控制，輸入 tool_configuration 包含末端執行器的六個自由度：位置（X, Y, Z）和姿態（roll, pitch, yaw）。
   方法內部通過逆運動學計算得到各關節的目標位置，然後使用 move_j 進行運動控制。
    '''
    def move_j_p(self, tool_configuration, k_acc=1, k_vel=1, t=0, r=0):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        print(f"movej_p([{tool_configuration}])")
        # command: movej([joint_configuration],a,v,t,r)\n
        tcp_command = "def process():\n"
        tcp_command += " array = rpy2rotvec([%f,%f,%f])\n" % (
            tool_configuration[3], tool_configuration[4], tool_configuration[5])
        tcp_command += "movej(get_inverse_kin(p[%f,%f,%f,array[0],array[1],array[2]]),a=%f,v=%f,t=%f,r=%f)\n" % (tool_configuration[0], tool_configuration[1], tool_configuration[2], k_acc * self.joint_acc, k_vel * self.joint_vel, t, r)  # "movej([]),a=,v=,\n"
        tcp_command += "end\n"
        self.tcp_socket.send(str.encode(tcp_command))

        # Block until robot reaches home state
        state_data = self.tcp_socket.recv(1500)
        actual_tool_positions = self.parse_tcp_state_data(
            state_data, 'cartesian_info')
        while not all([np.abs(actual_tool_positions[j] - tool_configuration[j]) < self.tool_pose_tolerance[j] for j in
                       range(3)]):
            state_data = self.tcp_socket.recv(1500)
            # print(f"tool_position_error{actual_tool_positions - tool_configuration}")
            actual_tool_positions = self.parse_tcp_state_data(
                state_data, 'cartesian_info')
            time.sleep(0.01)
        time.sleep(1.5)
        self.tcp_socket.close()

    # move_l is mean that the robot keep running in a straight line
    def move_l(self, tool_configuration, k_acc=1, k_vel=1, t=0, r=0):
        print(f"movel([{tool_configuration}])")
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        # command: movel([tool_configuration],a,v,t,r)\n
        tcp_command = "def process():\n"
        tcp_command += " array = rpy2rotvec([%f,%f,%f])\n" % (
            tool_configuration[3], tool_configuration[4], tool_configuration[5])
        tcp_command += "movel(p[%f,%f,%f,array[0],array[1],array[2]],a=%f,v=%f,t=%f,r=%f)\n" % (
            tool_configuration[0], tool_configuration[1], tool_configuration[2],
            k_acc * self.joint_acc, k_vel * self.joint_vel, t, r)  # "movel([]),a=,v=,\n"
        tcp_command += "end\n"
        self.tcp_socket.send(str.encode(tcp_command))

        # Block until robot reaches home state
        state_data = self.tcp_socket.recv(1500)
        actual_tool_positions = self.parse_tcp_state_data(
            state_data, 'cartesian_info')
        while not all([np.abs(actual_tool_positions[j] - tool_configuration[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
            state_data = self.tcp_socket.recv(1500)
            actual_tool_positions = self.parse_tcp_state_data(
                state_data, 'cartesian_info')
            time.sleep(0.01)
        time.sleep(1.5)
        self.tcp_socket.close()

    # Usually, We don't use move_c
    # move_c is mean that the robot move circle
    # mode 0: Unconstrained mode. Interpolate orientation from current pose to target pose (pose_to)
    #      1: Fixed mode. Keep orientation constant relative to the tangent of the circular arc (starting from current pose)
    def move_c(self, pose_via, tool_configuration, k_acc=1, k_vel=1, r=0, mode=0):

        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        print(f"movec([{pose_via},{tool_configuration}])")
        # command: movec([pose_via,tool_configuration],a,v,t,r)\n
        tcp_command = "def process():\n"
        tcp_command += " via_pose = rpy2rotvec([%f,%f,%f])\n" % (
            pose_via[3], pose_via[4], pose_via[5])
        tcp_command += " tool_pose = rpy2rot
