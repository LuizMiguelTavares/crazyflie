#!/usr/bin/env python3
from threading import Lock

import numpy as np
import rosbag
import rospy
import tf.transformations as tf_trans
from aurora_py.controllers import FeedForwardLQR
from aurora_py.kalman_filter import ExtendedKalmanFilter as EKF
from crazyflie_msgs.msg import CrazyflieLog
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Vector3Stamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from std_srvs.srv import Empty


class CrazyflieLQRNode:
    def __init__(self):
        rospy.init_node("crazyflie_LQR_node")

        ############## Crazyflie Model ############
        self.m = 0.035  # mass of the drone in kg
        self.g = 9.81  # gravity
        self.Kp = 1  # Proportional gain
        self.a_max = 17
        kvx = 0.03
        kvy = 0.03
        kvz = 0

        self.Kvx, self.Kvy, self.Kvz = kvx, kvy, kvz

        self.ang_max = rospy.get_param("~ang_max", 15.0)
        self.is_LQG = rospy.get_param("~is_LQG", False)
        self.use_orientation_on_pose = rospy.get_param(
            "~use_orientation_on_pose", False
        )
        pose_topic = rospy.get_param("~pose_topic", "/natnet_ros/cf1/pose")
        orientation_topic = rospy.get_param("~orientation_topic", "/cf1/crazyflieAng")
        
        self.circle_flag = False
        self.psi_ref_max = 100
        self.desired_z_height = 1
        self.ang_max_rad = np.deg2rad(self.ang_max)
        self.dont_takeoff = rospy.get_param("~dont_takeoff", False)
        self.print_kalman = rospy.get_param("~print_kalman", False)
        self.theta_trim = rospy.get_param("~theta_trim", 0.0)
        self.phi_trim = rospy.get_param("~phi_trim", 0.0)
        self.print_controller = rospy.get_param("~print_controller", True)
        # self.bag = rosbag.Bag(
        #     "/home/miguel/catkin_ws/src/crazyflie/crazyflie_controller/src/data/Testes.bag",
        #     "w",
        # )

        # Matrices A, B, C, D
        self.A = np.array(
            [[-kvx / self.m, 0, 0], [0, -kvy / self.m, 0], [0, 0, -kvz / self.m]]
        )

        self.B = np.array([[self.g, 0, 0], [0, self.g, 0], [0, 0, 1 / self.m]])

        self.C = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        self.D = np.zeros((self.C.shape[0], self.B.shape[1]))

        # LQR Matricies
        # self.Q = np.array([[10, 0, 0],
        #                     [0, 10, 0],
        #                     [0, 0, 1]])

        # self.R = np.array([[25, 0, 0],
        #                    [0, 25, 0],
        #                    [0, 0, 40]])

        # For Waypoints
        self.Q = np.array([[10, 0, 0], [0, 10, 0], [0, 0, 1]])

        self.R = np.array([[60, 0, 0], [0, 60, 0], [0, 0, 70]])

        ############## Kalman Filter ##########################
        self.Rw = np.diag([0.01, 0.01, 0.01, 0.2, 0.2, 0.2])  # Process noise covariance
        self.Rv = np.diag(
            [0.8, 0.8, 0.01, 0.01, 0.01, 0.01]
        )  # Measurement noise covariance

        self.initial_error_covariance = np.array(
            [
                [0.0, 0, 0, 0, 0, 0],
                [0, 0.0, 0, 0, 0, 0],
                [0, 0, 0.0, 0, 0, 0],
                [0, 0, 0, 0.0, 0, 0],
                [0, 0, 0, 0, 0.0, 0],
                [0, 0, 0, 0, 0, 0.0],
            ]
        )
        ########################################################

        # Flags
        self.emergency_button_pressed = False
        self.is_landing = False
        self.goto_center = False
        self.compute_controller_flag = False

        # Is true Flags
        self.compute_controller_is_true = False
        self.emergency_flag_is_true = False
        self.pose_is_true = False
        self.joy_is_true = False
        # self.crazyflie_LOG_flag = False
        self.crazyflie_angle_is_true = False
        self.crazyflie_vel_is_true = False
        self.crazyflie_thrust_is_true = False

        # Message to stop the drone
        self.stop_msg = Twist()
        self.stop_msg.linear.x = 0
        self.stop_msg.linear.y = 0
        self.stop_msg.linear.z = 0
        self.stop_msg.angular.z = 0

        self.namespace = rospy.get_namespace()

        # Subscriber for pose
        self.pose_subscriber = rospy.Subscriber(
            pose_topic, PoseStamped, self.pose_callback
        )
        
        self.orientation_subscriber = rospy.Subscriber(
            orientation_topic, Vector3Stamped, self.crazyflie_orientation_callback
        )
        
        self.velocity_subscriber = rospy.Subscriber(
            "/cf1/crazyflieVel", Vector3Stamped, self.crazyflie_vel_callback
        )
        
        self.thrust_subscriber = rospy.Subscriber(
            "/cf1/crazyflieThrust", Vector3Stamped, self.crazyflie_thrust_callback
        )

        # Subscriber for the joystick
        self.joystick_cmd = rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.control_publish = rospy.Publisher(f"cmd_vel", Twist, queue_size=10)

        self.trajectory_service = rospy.Service(
            "start_trajectory", Empty, self.handle_trajectory
        )

        # self.crazyflie_LOG = rospy.Subscriber(
        #     "crazyflieLog", CrazyflieLog, self.crazyflie_log_callback
        # )

        self.hz = 1 / 30
        self.rate = rospy.Rate(1 / self.hz)

        self.wait_rate = rospy.Rate(2)
        self.timer = rospy.Timer(rospy.Duration(1 / 4), self.prints)
        # self.bag_timer = rospy.Timer(rospy.Duration(self.hz), self.save_bag_timer)
        # self.bag_write_lock = Lock()
        self.is_bag_open = True
        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("Crazyflie LQG Node started")

    # def crazyflie_log_callback(self, msg):
    #     self.crazyflie_LOG_flag = True
    #     self.Log_msg = msg
    #     self.vx = msg.vx
    #     self.vy = msg.vy
    #     self.vz = msg.vz
    #     self.z = msg.z
    #     self.pitch = -msg.pitch
    #     self.roll = msg.roll
    #     self.yaw = msg.yaw
    #     self.true_thrust = msg.thrust
         
    def crazyflie_orientation_callback(self, msg):
        self.crazyflie_angle_is_true = True
        self.crazyflie_orientation = msg
        self.roll = msg.vector.x
        self.pitch = msg.vector.y
        self.yaw = msg.vector.z
    
    def crazyflie_vel_callback(self, msg):
        self.crazyflie_vel_is_true = True
        self.crazyflie_vel = msg
        self.vx = msg.vector.x
        self.vy = msg.vector.y
        self.vz = msg.vector.z
        
    def crazyflie_thrust_callback(self, msg):
        self.crazyflie_thrust_is_true = True
        self.crazyflie_thrust = msg
        self.true_thrust = msg.vector.z
        
    def shutdown_hook(self):
        with self.bag_write_lock:
            self.is_bag_open = False
            if self.bag is not None:
                self.bag.close()
            rospy.loginfo("ROS bag closed safely.")

    def get_euler_from_quaternion(self, quaternion):
        return np.array(
            tf_trans.euler_from_quaternion(
                [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            )
        )

    def emergency_landing(self, message):
        rospy.logerr(message)
        self.is_landing = True
        for _ in range(30):
            self.control_publish.publish(self.stop_msg)
        rospy.signal_shutdown("Drone Landed.")

    def joy_callback(self, data):
        if not self.emergency_button_pressed:
            self.joy_is_true = True
            self.joy = data

            if data.buttons[2]:
                self.goto_center = True
            if data.buttons[0]:
                self.emergency_button_pressed = True
                self.emergency_landing("Emergency landing")

    def pose_callback(self, msg):
        if not self.pose_is_true:
            self.pose_is_true = True
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        roll, pitch, yaw = self.get_euler_from_quaternion(msg.pose.orientation)

        self.optitrack_pose = msg

        self.pose = np.array([x, y, z, roll, pitch, yaw])
        self.pose_time = msg.header.stamp

    def conversions_crazyflie(self, theta, phi, thrust):
        # Normalizing angles and conversion to degrees
        u_theta = theta / self.ang_max_rad
        u_phi = phi / self.ang_max_rad

        theta_deg = (
            np.sign(u_theta) * self.ang_max
            if np.abs(u_theta) > 1.0
            else u_theta * self.ang_max
        )
        phi_deg = (
            np.sign(u_phi) * self.ang_max
            if np.abs(u_phi) > 1.0
            else u_phi * self.ang_max
        )

        # transforming thrust
        conversao_digital = 60000 / self.a_max

        digital_thrust = (thrust / self.m + self.g) * conversao_digital
        digital_thrust = min(max(digital_thrust, 11000), 60000)

        return np.array([theta_deg, phi_deg, digital_thrust])

    def undo_conversions_crazyflie(self, theta, phi, thrust):
        # Normalizing angles and conversion to degrees
        theta_rad = np.deg2rad(theta)
        phi_rad = np.deg2rad(phi)

        # transforming thrust
        conversao_digital = 60000 / self.a_max
        thrust = (thrust * self.m - self.g) / conversao_digital

        return theta_rad, phi_rad, thrust

    def compute_psi(self, psi_dot_ref, psi_desired):
        psi_til = psi_desired - self.pose[5]
        if np.abs(psi_til) > np.pi:
            if psi_til > 0:
                psi_til = psi_til - 2 * np.pi
            else:
                psi_til = psi_til + 2 * np.pi

        psi_ref = psi_dot_ref + 3 * psi_til
        psi_ref = min(max(psi_ref, -1), 1)
        return psi_ref * self.psi_ref_max

    def x_dot_ref(self, x_dot_desired, x_til):
        return x_dot_desired + self.Kp * x_til

    def rotation_world_to_body(self, orientation, vector):
        R = np.array(
            [
                [np.cos(orientation), np.sin(orientation)],
                [-np.sin(orientation), np.cos(orientation)],
            ]
        )
        return np.dot(R, vector)

    def compute_controller(self):
        counter = 0
        kalman_flag = False
        first_loop = True

        cf_LQR = FeedForwardLQR(self.A, self.B, self.C, self.D, self.Q, self.R)

        while not rospy.is_shutdown():
            if not self.compute_controller_flag:
                self.rate.sleep()
                continue
                
            if not self.crazyflie_angle_is_true or not self.crazyflie_vel_is_true or not self.crazyflie_thrust_is_true:
                self.rate.sleep()
                if not self.crazyflie_angle_is_true:
                    rospy.loginfo("Waiting for the crazyflie angle!")
                if not self.crazyflie_vel_is_true:
                    rospy.loginfo("Waiting for the crazyflie velocity!")
                if not self.crazyflie_thrust_is_true:
                    rospy.loginfo("Waiting for the crazyflie thrust!")                    
                continue

            if not self.pose_is_true:
                counter += 1
                if counter == 4:
                    rospy.loginfo(
                        "It was'nt possible to find the pose of the crazyflie!"
                    )
                    counter = 0
                self.wait_rate.sleep()
                continue
            
            if self.is_landing:
                self.rate.sleep()
                continue
            
            if not self.use_orientation_on_pose:
                self.pose[3] = self.roll
                self.pose[4] = self.pitch
                self.pose[5] = self.yaw
            
            pose = self.pose
            pose_time = self.pose_time

            if first_loop:
                ### Takeoff
                takeoff_time = rospy.Time.now()
                self.desired_takeoff_pose = [
                    self.pose[0],
                    self.pose[1],
                    self.pose[2] + self.desired_z_height,
                    0,
                    0,
                    0,
                ]

                ### Kalman Filter
                kalman_time = rospy.Time.now()
                initial_state = np.array([pose[0], pose[1], pose[2], 0, 0, 0])
                kalman_filter = EKF(
                    initial_state,
                    self.initial_error_covariance,
                    self.hz,
                    self.m,
                    self.Kvx,
                    self.Kvy,
                    self.Kvz,
                    self.Rw,
                    self.Rv,
                )
                self.state_estimate = initial_state

                ### Velocity
                vel = np.array([0.0, 0.0, 0.0])
                vel_pose_old = pose
                vel_time_old = pose_time

                first_loop = False

            # Get the current timestamp
            current_time = rospy.Time.now()

            ### Kalman Filter
            elapsed_kalman_time = (current_time - kalman_time).to_sec()

            if (elapsed_kalman_time >= self.hz) | (not kalman_flag):
                kalman_time = current_time

                noise = 0.2
                x_gaussian_error = pose[0] + np.random.normal(0, noise)
                y_gaussian_error = pose[1] + np.random.normal(0, noise)
                z_gaussian_error = pose[2]

                kalman_control_input_ = self.undo_conversions_crazyflie(
                    self.pitch, self.roll, self.true_thrust
                )

                yaw = self.yaw * np.pi / 180
                self.kalman_control_input = np.array(
                    [
                        kalman_control_input_[0],
                        kalman_control_input_[1],
                        yaw,
                        kalman_control_input_[2],
                    ]
                )

                vx_vy = np.array([self.vx, self.vy])

                vx_vy_body = self.rotation_world_to_body(yaw, vx_vy)

                self.position_with_gaussian_error = np.array(
                    [
                        x_gaussian_error,
                        y_gaussian_error,
                        z_gaussian_error,
                        vx_vy_body[0],
                        vx_vy_body[1],
                        self.vz,
                    ]
                )
                try:
                    self.state_estimate, self.kalman_gain = kalman_filter.update(
                        self.position_with_gaussian_error, self.kalman_control_input
                    )
                except:
                    pass

                if kalman_flag:
                    # Compute velocity
                    dt = (pose_time - vel_time_old).to_sec()
                    if dt == 0:
                        self.rate.sleep()
                        continue

                    vel = (
                        np.array([pose[0], pose[1], pose[2]])
                        - np.array([vel_pose_old[0], vel_pose_old[1], vel_pose_old[2]])
                    ) / dt

                    # print(f'Rate: {np.round(1/dt, 3)}')

                    vel_pose_old = pose
                    vel_time_old = pose_time

                kalman_flag = True

            # Calculate elapsed time
            elapsed_time = (current_time - takeoff_time).to_sec()
            self.elapsed_time = elapsed_time

            angular_velocity = np.sqrt(2)
            x_radius = 1
            y_radius = 1
            z_radius = 0.25

            wait_time = 3

            if elapsed_time < wait_time:
                psi_desired = 0
                psi_dot_ref = 0

                x_til = self.desired_takeoff_pose[:3] - pose[:3]
                desired_position_velocity = self.desired_takeoff_pose
                desired = self.x_dot_ref(np.zeros(3), x_til)

            else:
                if not self.circle_flag:
                    self.circle_flag = True
                    self.elapsed_circle_time = 0
                    self.circle_time = rospy.Time.now().to_sec()

                ### Circle
                x = x_radius * np.cos(angular_velocity * elapsed_time)
                y = y_radius * np.sin(angular_velocity * elapsed_time)
                z = self.desired_z_height
                
                vx = (
                    -x_radius
                    * angular_velocity
                    * np.sin(angular_velocity * elapsed_time)
                )
                vy = (
                    y_radius
                    * angular_velocity
                    * np.cos(angular_velocity * elapsed_time)
                )
                vz = 0
                
                psi_desired = np.arctan2(vy, vx)
                psi_dot_ref = angular_velocity

                # # Lemniscata
                # x = x_radius * np.cos(angular_velocity * elapsed_time)
                # y = y_radius * np.sin(2 * angular_velocity * elapsed_time)
                # z = self.desired_z_height + z_radius * np.cos(
                #     angular_velocity * elapsed_time
                # )
                #
                # vx = (
                #     -x_radius
                #     * angular_velocity
                #     * np.sin(angular_velocity * elapsed_time)
                # )
                # vy = (
                #     2
                #     * y_radius
                #     * angular_velocity
                #     * np.cos(2 * angular_velocity * elapsed_time)
                # )
                # vz = (
                #     -z_radius
                #     * angular_velocity
                #     * np.sin(angular_velocity * elapsed_time)
                # )
                #
                # psi_desired = np.arctan2(vy, vx)
                # psi_dot_ref = angular_velocity

                # # Pringles
                # x = x_radius * np.cos(angular_velocity * elapsed_time)
                # y = y_radius * np.sin(angular_velocity * elapsed_time)
                # z = self.desired_z_height + z_radius * np.cos(2 * angular_velocity * elapsed_time)

                # vx = -x_radius * angular_velocity * np.sin(angular_velocity * elapsed_time)
                # vy = y_radius * angular_velocity * np.cos(angular_velocity * elapsed_time)
                # vz = - 2 * z_radius * angular_velocity * np.sin(2 * angular_velocity * elapsed_time)

                # # # Waypoints

                # waypoints = np.array(
                #     [
                #         [-1.8, 1.2, 0.7],
                #         [2.2, -1.2, 1.3],
                #         [-1.8, -1.2, 0.7],
                #         [2.2, 1.2, 1.3],
                #     ]
                # )  # Rain
                # waypoints = np.array(
                #     [[-2, 1.2, 0.8], [2, -1.2, 1.5], [-2, -1.2, 0.8], [2, 1.2, 1.5]]
                # )  # 0fficial
                # waypoints = np.array(
                #     [[-1, 1, 0.8], [1.5, -1, 1.5], [-1, -1, 0.8], [1.5, 1, 1.5]]
                # )  # Camera

                # waypoint_time = 6

                # if self.elapsed_circle_time < waypoint_time:
                #     x = waypoints[0, 0]
                #     y = waypoints[0, 1]
                #     z = waypoints[0, 2]

                #     vx = 0
                #     vy = 0
                #     vz = 0
                # elif self.elapsed_circle_time < 2 * waypoint_time:
                #     x = waypoints[1, 0]
                #     y = waypoints[1, 1]
                #     z = waypoints[1, 2]

                #     vx = 0
                #     vy = 0
                #     vz = 0
                # elif self.elapsed_circle_time < 3 * waypoint_time:
                #     x = waypoints[2, 0]
                #     y = waypoints[2, 1]
                #     z = waypoints[2, 2]

                #     vx = 0
                #     vy = 0
                #     vz = 0
                # elif self.elapsed_circle_time < 4 * waypoint_time:
                #     x = waypoints[3, 0]
                #     y = waypoints[3, 1]
                #     z = waypoints[3, 2]

                #     vx = 0
                #     vy = 0
                #     vz = 0
                # else:
                #     x = waypoints[0, 0]
                #     y = waypoints[0, 1]
                #     z = waypoints[0, 2]

                #     vx = 0
                #     vy = 0
                #     vz = 0

                # psi_desired = 0
                # psi_dot_ref = 0

                # # # Eight
                # centers = [-1, 1]
                # x_radius = 1
                # y_radius = 1
                # x = x_radius * np.cos(angular_velocity * elapsed_time)
                # y = y_radius * np.sin(2*angular_velocity * elapsed_time)
                # z = self.desired_z_height

                # # Line
                # x = x_radius * np.cos(angular_velocity * elapsed_time)
                # y = 0.0
                # z = self.desired_z_height

                # vx = -x_radius * angular_velocity * np.sin(angular_velocity * elapsed_time)
                # vy = 0
                # vz = 0

                desired_position_velocity = np.array([x, y, z, vx, vy, vz])

                if self.is_LQG:
                    x_til = np.array([x, y, z]) - self.state_estimate[:3]
                else:
                    x_til = np.array([x, y, z]) - pose[:3]

                desired = self.x_dot_ref(np.array([vx, vy, vz]), x_til)

                # ax = -x_radius * angular_velocity**2 * np.cos(angular_velocity * elapsed_time)
                # ay = -y_radius * angular_velocity**2 * np.sin(angular_velocity * elapsed_time)
                # az = 0
            self.goto_center = True
            if self.goto_center:
                desired_position_velocity = np.array(
                    [0, 0, self.desired_z_height, 0, 0, 0]
                )
                x_til = np.array([0, 0, self.desired_z_height]) - pose[:3]
                desired = self.x_dot_ref(np.zeros(3), x_til)

                psi_desired = 0
                psi_dot_ref = 0

            self.pose_LQR = np.array(
                [pose[0], pose[1], pose[2], vel[0], vel[1], vel[2]]
            )
            ### Vel optitrack
            # vel = np.array([vel[0], vel[1], vel[2]])

            ### Vel crazuflie
            vel = np.array([self.vx, self.vy, self.vz])

            # Compute LQR controller
            rotation_matrix_theta_phi = np.array(
                [
                    [np.cos(pose[5]), np.sin(pose[5])],
                    [-np.sin(pose[5]), np.cos(pose[5])],
                ]
            )

            theta_phi = rotation_matrix_theta_phi @ np.array([vel[0], vel[1]])
            new_vel = np.array([theta_phi[0], theta_phi[1], vel[2]])

            new_desired = rotation_matrix_theta_phi @ np.array([desired[0], desired[1]])
            self.reference_vel = np.array([new_desired[0], new_desired[1], desired[2]])

            ### LQG
            if self.circle_flag:
                self.elapsed_circle_time = rospy.Time.now().to_sec() - self.circle_time
                estimated_vel = self.state_estimate[3:]
                theta_LQR, phi_LQR, thrust_LQR = cf_LQR.compute_u(
                    estimated_vel, self.reference_vel
                )
                self.desired = desired_position_velocity
                # theta_LQR, phi_LQR, thrust_LQR = cf_LQR.compute_u(new_vel, self.reference_vel)
            else:
                theta_LQR, phi_LQR, thrust_LQR = cf_LQR.compute_u(
                    new_vel, self.reference_vel
                )

            self.raw_control_reference = np.array([theta_LQR, phi_LQR, thrust_LQR])

            theta_ref, phi_ref, thrust_ref = self.conversions_crazyflie(
                theta_LQR, phi_LQR, thrust_LQR
            )
            psi_ref = self.compute_psi(psi_dot_ref, psi_desired)

            self.theta_ref = theta_ref + self.theta_trim
            self.phi_ref = phi_ref + self.phi_trim
            self.psi_ref = psi_ref
            self.thrust_ref = thrust_ref

            self.compute_controller_is_true = True

    def prints(self, req):
        
        if not self.compute_controller_is_true:
            return

        if self.print_kalman:
            print(f"Measure : {np.round(self.pose_LQR, 3)}")
            try:
                print(f"Kalman: {np.round(self.state_estimate, 3)}")
            except:
                pass

            print()

        if self.print_controller:
            print(
                f"LQR_controller : {[float(np.round(val, 3)) for val in [self.theta_ref, -self.phi_ref, self.psi_ref, self.thrust_ref/60000]]}"
            )

    def save_bag_timer(self, req):
        if not self.compute_controller_is_true or not self.is_bag_open:
            return

        with self.bag_write_lock:
            # if self.circle_flag:
            write_flag = True

            if not self.is_bag_open:
                return

            ### time
            try:
                time = Float32()
                # time.data = self.elapsed_circle_time
                time.data = self.elapsed_time

                ### state
                pose_LQR_msg = Vector3()
                pose_LQR_msg.x, pose_LQR_msg.y, pose_LQR_msg.z = self.pose_LQR[:3]

                vel_LQR_msg = Vector3()
                vel_LQR_msg.x, vel_LQR_msg.y, vel_LQR_msg.z = self.pose_LQR[3:]

                ### state estimate
                position_estimate_msg = Vector3()
                (
                    position_estimate_msg.x,
                    position_estimate_msg.y,
                    position_estimate_msg.z,
                ) = self.state_estimate[:3]
                vel_estimate_msg = Vector3()
                vel_estimate_msg.x, vel_estimate_msg.y, vel_estimate_msg.z = (
                    self.state_estimate[3:]
                )

                ### Desired
                desired_position_msg = Vector3()
                (
                    desired_position_msg.x,
                    desired_position_msg.y,
                    desired_position_msg.z,
                ) = self.desired[:3]
                desired_vel_msg = Vector3()
                desired_vel_msg.x, desired_vel_msg.y, desired_vel_msg.z = self.desired[
                    3:
                ]

                ### Reference velocity
                ref_vel_msg = Vector3()
                ref_vel_msg.x, ref_vel_msg.y, ref_vel_msg.z = self.reference_vel

                ### Position with gaussian error
                position_gaussian_error_msg = Vector3()
                (
                    position_gaussian_error_msg.x,
                    position_gaussian_error_msg.y,
                    position_gaussian_error_msg.z,
                ) = self.position_with_gaussian_error[:3]

                ### Control input ref
                control_input_msg = Twist()
                (
                    control_input_msg.linear.x,
                    control_input_msg.linear.y,
                    control_input_msg.angular.z,
                    control_input_msg.linear.z,
                ) = self.kalman_control_input

                ### Raw control reference
                raw_control_reference_msg = Vector3()
                (
                    raw_control_reference_msg.x,
                    raw_control_reference_msg.y,
                    raw_control_reference_msg.z,
                ) = self.raw_control_reference

                ### Control sent
                control_sent_msg = Twist()
                (
                    control_sent_msg.linear.x,
                    control_sent_msg.linear.y,
                    control_sent_msg.angular.z,
                    control_sent_msg.linear.z,
                ) = (self.theta_ref, self.phi_ref, self.psi_ref, self.thrust_ref)

                ### Log
                # log_msg = self.Log_msg
                
                crazyflie_vel_msg = self.crazyflie_vel
                crazyflie_thrust_msg = self.crazyflie_thrust
                crazyflie_angle_msg = self.crazyflie_orientation

                ### Optitrack
                optitrack_msg = self.optitrack_pose
            except:
                write_flag = False

            ## write_bag
            if write_flag:
                self.bag.write("time", time)
                self.bag.write("position_Optitrack", pose_LQR_msg)
                self.bag.write("vel_Optitrack", vel_LQR_msg)
                self.bag.write("position_estimate_topic", position_estimate_msg)
                self.bag.write("vel_estimate_topic", vel_estimate_msg)
                self.bag.write("desired_position", desired_position_msg)
                self.bag.write("desired_vel", desired_vel_msg)
                self.bag.write("reference_vel", ref_vel_msg)
                self.bag.write("position_gaussian_error", position_gaussian_error_msg)
                self.bag.write("control_input", control_input_msg)
                self.bag.write("raw_control_reference", raw_control_reference_msg)
                self.bag.write("control_sent", control_sent_msg)
                self.bag.write("crazyflie_vel", crazyflie_vel_msg)
                self.bag.write("crazyflie_thrust", crazyflie_thrust_msg)
                self.bag.write("crazyflie_angle", crazyflie_angle_msg)
                # self.bag.write("crazyflieLog", log_msg)
                self.bag.write("optitrack_pose", optitrack_msg)
            else:
                write_flag = True

    def handle_trajectory(self, req):
        self.compute_controller_flag = True
        idle_time = 3
        self.idle_flag = False
        
        while not rospy.is_shutdown():
            if self.is_landing:
                self.rate.sleep()
                continue

            if not self.compute_controller_is_true:
                self.rate.sleep()
                continue

            theta_ref = self.theta_ref
            phi_ref = self.phi_ref
            psi_ref = self.psi_ref
            thrust_ref = self.thrust_ref

            ### Joystick
            if self.joy_is_true:
                if np.linalg.norm([self.joy.axes[4], self.joy.axes[3]]) > 0.1:
                    theta_ref = self.joy.axes[4] * self.ang_max
                    phi_ref = self.joy.axes[3] * self.ang_max

                if abs((self.joy.axes[2] - self.joy.axes[5]) * 0.5) > 0.1:
                    psi_ref = -((self.joy.axes[2] - self.joy.axes[5]) * 0.5) * 100

                if abs(self.joy.axes[1]) > 0.1:
                    thrust_ref = (0.5 + self.joy.axes[1] * 0.5) * 60000
                    thrust_ref = min(max(thrust_ref, 40000), 60000)

            pub_controller = Twist()
            pub_controller.angular.x = np.deg2rad(-phi_ref)
            pub_controller.angular.y = np.deg2rad(theta_ref)
            pub_controller.linear.z = thrust_ref
            pub_controller.angular.z = np.deg2rad(psi_ref)
            if not self.dont_takeoff:
                if not self.idle_flag:
                    self.idle_initial_time = rospy.Time.now().to_sec()
                    self.idle_flag = True
                
                if rospy.Time.now().to_sec() - self.idle_initial_time < idle_time:
                    pub_idle = Twist()
                    pub_idle.linear.z = 12000
                    self.control_publish.publish(pub_idle)
                    self.rate.sleep()
                else:
                    self.control_publish.publish(pub_controller)

            self.rate.sleep()


if __name__ == "__main__":
    node = CrazyflieLQRNode()
    try:
        node.compute_controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(e)
        rospy.signal_shutdown("Parameter not set")

