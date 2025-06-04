#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, ActuatorMotors, VehicleAttitude, VehicleOdometry
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Vector3

class AttitudePDController:
    
    def __init__(self, kp, kd):
        """
        kp, kd: listy 3-elementowe [roll, pitch, yaw]
        base_thrust: stały ciąg (np. 1500 µs na ESC)
        """
        self.kp = kp
        self.kd = kd
        # self.prev_error = [0.0, 0.0, 0.0]

    def update(self, desired_angles, current_angles, angular_velocities):
        """
        desired_angles: [roll, pitch, yaw] w stopniach lub radianach
        current_angles: [roll, pitch, yaw]
        angular_velocities: prędkości kątowe w poszczególnych osiach
        """

        errors = [d - c for d, c in zip(desired_angles, current_angles)]
        for i in range(3):
            if errors[i] > 180:
                errors[i] -= 360
            if errors[i] < -180:
                errors[i] += 360
        d_errors = [float(-w) for w in angular_velocities]  # zamiast różniczki
        moments = [
            self.kp[i] * errors[i] + self.kd[i] * d_errors[i]
            for i in range(3)
        ]

        print(f"errors: {errors} | d_errors: {d_errors}")

        return moments, errors


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.actuator_motors_publisher = self.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)
        self.moments = self.create_publisher(Vector3, 'moments', 10)
        self.angles = self.create_publisher(Vector3, 'angles', 10)
        self.errors = self.create_publisher(Vector3, 'errors', 10)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.vehicle_attitude = []
        self.angular_velocity = []
        self.control_flag = 0
        # Create a timer to publish control commands
        self.timer = self.create_timer(0.005, self.timer_callback)

    def vehicle_attitude_callback(self, vehicle_attitude: VehicleAttitude):
        """Callback function for vehicle_attitude topic subscriber."""

        r = R.from_quat([vehicle_attitude.q[1], vehicle_attitude.q[2], vehicle_attitude.q[3], vehicle_attitude.q[0]])
        r = r.as_euler('ZXY', degrees=True)
        self.vehicle_attitude = [r[1], r[2], r[0]]

    def vehicle_odometry_callback(self, vehicle_odometry: VehicleOdometry):
        """Callback function for vehicle_odometry topic subscriber."""
    
        self.angular_velocity = vehicle_odometry.angular_velocity

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_actuator_motors(self, motors_vel):
        """Publish actutator_motors."""
        msg = ActuatorMotors()
        msg.control = [motors_vel[0], motors_vel[1], motors_vel[2], motors_vel[3], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.actuator_motors_publisher.publish(msg)

    def publish_debug(self, moments, angles, errors):
        msg = Vector3()
        msg.x = moments[0]
        msg.y = moments[1]
        msg.z = moments[2]
        self.moments.publish(msg)

        msg2 = Vector3()
        msg2.x = angles[0]
        msg2.y = angles[1]
        msg2.z = angles[2]
        self.angles.publish(msg2)

        msg3 = Vector3()
        msg3.x = errors[0]
        msg3.y = errors[1]
        msg3.z = errors[2]
        self.errors.publish(msg3)


    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def angle_controller(self, desired):

        controller = AttitudePDController(
            kp=[0.02, 0.02, 0.02],  # roll, pitch, yaw
            kd=[0.0, 0.0, 0.0])

        moments, errors = controller.update(desired, self.vehicle_attitude, self.angular_velocity)
        print(f"angles: {self.vehicle_attitude}")
        self.publish_debug(moments, self.vehicle_attitude, errors)

        print("Moment roll/pitch/yaw:", moments)
        thrust = 20
        final_vector = np.array([thrust, moments[0], moments[1], moments[2]])
        print(final_vector.shape)
        ct = 8.54858e-06  # c_T
        cq = 8.06428e-05  # c_Q
        p = 0.174
        # Obliczenie d = sqrt(0.174^2 + 0.174^2)
        # d = np.sqrt(0.174**2 + 0.174**2)

        # Macierz C z równania (8)
        C = np.array([
            [ct, ct, ct, ct],
            [-p*ct, p*ct, p*ct, -p*ct],
            [p*ct, -p*ct, p*ct, -p*ct],
            [cq, cq, -cq, -cq]
        ])
        c_inv = np.linalg.inv(C)
        ang_vel_motor = c_inv @ final_vector.T
        ang_vel_motor_sqrt = np.sqrt(ang_vel_motor) / 1000
        print(f"ang_vel_motor_sqrt: {ang_vel_motor_sqrt}")
        return ang_vel_motor_sqrt

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 200:
            self.engage_offboard_mode()
            self.arm()

        # if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.control_flag==0:
        #     self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
        if self.offboard_setpoint_counter < 400 and self.offboard_setpoint_counter > 200:
            self.publish_actuator_motors([0.75, 0.75, 0.75, 0.75])

        if self.offboard_setpoint_counter > 401:
            self.control_flag = 1
            print("change to control")
            desired = [0, 0, 80]
            ang_vel_motor_sqrt = self.angle_controller(desired)
            self.publish_actuator_motors(ang_vel_motor_sqrt)
        
        self.offboard_setpoint_counter += 1

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
