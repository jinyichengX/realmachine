#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
学习用 2D 激光栅格建图 Demo（纯手写核心算法，无黑盒）
⚠️ 重要定位说明：
  本代码仅实现「基于里程计的增量式栅格建图」，没有激光配准、位姿修正环节，
  不属于完整意义上的 SLAM（同时定位与建图），仅用于学习栅格地图的构建原理。

核心原理（无回环、无图优化）：
  1. 输入：/scan 激光数据 + TF(odom→激光坐标系) 里程机位姿
  2. 每帧激光执行三步：
     a. 坐标变换（手写2D旋转矩阵）：激光点从传感器坐标系转到世界坐标系
     b. 终点标记：激光命中点标记为占用（log-odds 加分）
     c. 射线标记：激光穿过的区域标记为空闲（log-odds 减分，手写Bresenham算法）
  3. 输出：/map 栅格地图话题 + 动态 map→odom TF，可直接对接RViz/Nav2

log-odds 对数几率说明：
  每个格子存储一个值，表示“有障碍物”的置信度：
  l > 0 → 大概率是障碍；l < 0 → 大概率空闲；l = 0 → 未探索
  每帧更新做加减法，发布时转为0~100的占用概率。
"""

import math
import numpy as np  # 仅作为二维数组容器，核心算法全部手写

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


class SimpleMappingNode(Node):
    """基于里程计的增量式2D激光建图节点（学习用）"""

    def __init__(self):
        super().__init__('simple_mapping')

        # ================= 地图参数 =================
        self.resolution = 0.05             # 栅格分辨率：5cm/格
        self.map_size_m = 20.0             # 地图尺寸：20m x 20m
        self.width = int(self.map_size_m / self.resolution)
        self.height = int(self.map_size_m / self.resolution)
        self.origin_x = -self.map_size_m / 2.0  # 地图原点在世界坐标中心
        self.origin_y = -self.map_size_m / 2.0

        # log-odds 栅格地图
        self.map_log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.log_odds_occ = 0.9    # 命中障碍：加分
        self.log_odds_free = -0.7  # 穿过空闲：减分
        self.clamp_min = -4.0      # 数值下限，防止无限累积
        self.clamp_max = 4.0       # 数值上限

        # ================= 话题订阅与发布 =================
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/map', 10)
        self.map_timer = self.create_timer(2.0, self.publish_map)  # 0.5Hz发布地图

        # ================= TF 相关 =================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 修复: 使用动态TF广播器发布 map→odom（符合ROS REP 105规范，必须动态发布）
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self._publish_map_odom_tf)  # 20Hz高频发布

        self.get_logger().info(
            f'简易建图节点启动：地图 {self.map_size_m}m x {self.map_size_m}m, '
            f'分辨率 {self.resolution}m, 栅格 {self.width}x{self.height}')

    # ==================================================
    # 主回调：每收到一帧激光执行一次
    # ==================================================
    def scan_callback(self, msg: LaserScan):
        # ---- 1. 查询当前激光帧时刻的位姿 ----
        try:
            # 修复: 使用激光帧自身的时间戳查询TF，保证数据与位姿时间同步
            t = self.tf_buffer.lookup_transform(
                'odom',
                msg.header.frame_id,#激光 frame_id 是 base_footprint
                msg.header.stamp,
                timeout=Duration(seconds=0.1)
            )
        except Exception:
            return  # TF未就绪/超时则跳过当前帧

        # 解析激光原点在odom坐标系下的位姿 (x, y, yaw)
        robot_x = t.transform.translation.x
        robot_y = t.transform.translation.y
        robot_yaw = self.quat_to_yaw(t.transform.rotation)

        # 机器人原点对应的栅格坐标（射线起点）
        robot_gx, robot_gy = self.world_to_map(robot_x, robot_y)

        # ---- 2. 遍历每一束激光 ----
        # 修复: 用索引计算角度，避免浮点数循环累加产生累积误差
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # 过滤无效距离数据
            if (math.isinf(r) or math.isnan(r)
                    or r < msg.range_min  # 修复: 过滤小于雷达最小有效距离的无效数据
                    or r >= msg.range_max):
                continue

            # 2.1 极坐标转激光坐标系下的直角坐标
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)

            # 2.2 手写2D旋转+平移，转到世界(odom)坐标系
            wx = robot_x + lx * math.cos(robot_yaw) - ly * math.sin(robot_yaw)
            wy = robot_y + lx * math.sin(robot_yaw) + ly * math.cos(robot_yaw)

            # 2.3 世界坐标转栅格坐标
            gx, gy = self.world_to_map(wx, wy)
            if not self.in_map(gx, gy):
                continue

            # 2.4 终点格子：标记为占用（障碍物）
            self.add_log_odds(gx, gy, self.log_odds_occ)

            # 2.5 射线格子：从机器人到终点，经过的区域标记为空闲
            self.draw_ray(robot_gx, robot_gy, gx, gy, self.log_odds_free)

    # ==================================================
    # 手写核心算法部分
    # ==================================================

    def quat_to_yaw(self, q):
        """四元数提取2D偏航角yaw（绕Z轴旋转），标准公式推导"""
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def world_to_map(self, wx, wy):
        """世界坐标(米) → 栅格坐标(整数格子编号)"""
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def in_map(self, gx, gy):
        """判断栅格坐标是否在地图范围内"""
        return 0 <= gx < self.width and 0 <= gy < self.height

    def add_log_odds(self, gx, gy, delta):
        """log-odds更新：l_new = clamp(l_old + delta)，栅格地图核心逻辑"""
        new_val = self.map_log_odds[gy, gx] + delta
        if new_val > self.clamp_max:
            new_val = self.clamp_max
        elif new_val < self.clamp_min:
            new_val = self.clamp_min
        self.map_log_odds[gy, gx] = new_val

    def draw_ray(self, x0, y0, x1, y1, value):
        """手写Bresenham直线算法，标记射线穿过的空闲格子
        注意：仅更新路径，终点（障碍物）由调用方单独标记
        """
        # 修复: 先标记起点（机器人自身位置）为空闲，符合物理事实
        if self.in_map(x0, y0):
            self.add_log_odds(x0, y0, value)

        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy  # 累计误差

        x, y = x0, y0
        while True:
            if x == x1 and y == y1:
                break  # 到达终点停止，终点不更新为空闲
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
            if self.in_map(x, y):
                self.add_log_odds(x, y, value)

    # ==================================================
    # 发布逻辑
    # ==================================================

    def publish_map(self):
        """将log-odds地图转为OccupancyGrid消息发布（0~100占用值，-1为未知）"""
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        # 修复: 显式设置合法单位四元数，避免Nav2、RViz校验失败
        msg.info.origin.orientation.w = 1.0

        # log-odds → 占用概率（sigmoid反变换）
        probs = 1.0 - 1.0 / (1.0 + np.exp(self.map_log_odds))
        # l=0为未探索区域，标记为-1；其余转为0~100的整数
        data = np.where(
            self.map_log_odds == 0.0, -1,
            (probs * 100.0).astype(np.int8)
        )
        msg.data = data.flatten().tolist()

        self.map_pub.publish(msg)

    def _publish_map_odom_tf(self):
        """动态发布 map→odom 变换（简化版为恒等变换，无位姿修正）
        规范说明：真实SLAM中，这个变换会随时间变化，用于抵消里程计漂移
        """
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'map'
        ts.child_frame_id = 'odom'
        ts.transform.rotation.w = 1.0  # 单位四元数=恒等旋转
        self.tf_broadcaster.sendTransform(ts)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMappingNode()
    # 多线程执行器：scan_callback 与发布定时器并行执行，避免回调互相饿死
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()