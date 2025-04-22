import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleAttitude
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from scipy.spatial.transform import Rotation as R
import numpy as np

class AttitudeToRPY(Node):
    def __init__(self):
        super().__init__('attitude_to_rpy_node')

        # Match PX4's QoS (BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.listener_callback,
            qos_profile
        )
        self.get_logger().info('Subscribed to /fmu/out/vehicle_attitude with BEST_EFFORT QoS')

    def listener_callback(self, msg):
        q = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])
        r = R.from_quat([q[1], q[2], q[3], q[0]])  # Convert to [x, y, z, w]
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        self.get_logger().info(f'Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°')


def main(args=None):
    rclpy.init(args=args)
    node = AttitudeToRPY()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()