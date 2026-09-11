import math
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

def quaternion_to_euler(x, y, z, w):
    """四元数转欧拉角(弧度)，返回(roll, pitch, yaw)，不依赖任何第三方库"""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - z * x))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return (roll, pitch, yaw)

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.lookup_tf)
        self.get_logger().info('TF 监听器已启动，查询 base_link -> bottle_link')

    def lookup_tf(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            # 四元数 → 欧拉角
            rx = t.transform.rotation.x
            ry = t.transform.rotation.y
            rz = t.transform.rotation.z
            rw = t.transform.rotation.w
            roll, pitch, yaw = quaternion_to_euler(rx, ry, rz, rw)

            self.get_logger().info(
                f'平移(m): x={t.transform.translation.x:.3f}, '
                f'y={t.transform.translation.y:.3f}, '
                f'z={t.transform.translation.z:.3f} | '
                f'旋转(°): roll={math.degrees(roll):.1f}, '
                f'pitch={math.degrees(pitch):.1f}, '
                f'yaw={math.degrees(yaw):.1f}'
            )
        except Exception as e:
            self.get_logger().warn(f'查询失败: {e}')

def main():
    rclpy.init()
    node = TFListener()
    rclpy.spin(node)
    rclpy.shutdown()