import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import math

class ArucoPoseSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_pose_subscriber')
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.listener_callback,
            10  # QoS参数
        )
        self.subscription  # 防止被垃圾回收

    def listener_callback(self, msg):
        self.get_logger().info('Received PoseArray message:')
        print(msg.header.frame_id)
        for pose in msg.poses:
            position = pose.position
            orientation = pose.orientation

            # 计算距离（假设参考点为原点）
            distance = math.sqrt(position.x**2 + position.y**2 + position.z**2)
            self.get_logger().info(f'  Distance: {distance:.2f}')
            z_value = orientation.z
            if orientation.z / orientation.x > 0:
                z_value = abs(z_value)
            else:
                z_value = -abs(z_value)
                
            if z_value > 0.05:
                print("打印机器头偏右")
            elif z_value < -0.05:
                print("打印机器头偏左")
            else:
                print("打印机器头居中")
 
            # 计算方向（使用欧拉角表示）
            # roll, pitch, yaw = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
            # self.get_logger().info(f'  Direction (yaw): {yaw:.2f} radians')

    def euler_from_quaternion(self, x, y, z, w):
        # 使用四元数转换为欧拉角
        import numpy as np
        q = np.array([w, x, y, z])
        roll = np.arctan2(2 * (q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))
        pitch = np.arcsin(2 * (q[0]*q[2] - q[3]*q[1]))
        yaw = np.arctan2(2 * (q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]**2 + q[3]**2))
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
