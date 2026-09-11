#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rikibot 真机底盘驱动节点 (ROS2)

协议参考: /root/auto_navigation/rikibot_chassis_rosserial.md
底盘: 锐趣科技 STM32F103RC rosserial 板, 4WD 麦轮速度闭环底盘

功能:
  1. 订阅 /cmd_vel(geometry_msgs/Twist) -> 50Hz 转发底盘 (topic: cmd_vel/Twist)
     - 安全: 超过 timeout 收不到 cmd_vel 自动下发零速
  2. 读 raw_vel(riki_msgs/Velocities) 50Hz -> 麦轮三通道积分 -> /odom + odom->base_link TF
  3. 转发 raw_imu -> /imu_raw, battery -> /battery

用法:
  ros2 run rikibot_bringup rikibot_base_node
"""
import glob
import math
import os
import struct
import time

import rclpy
import serial
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import tf2_ros

# ---------------- rosserial 协议 (帧格式见 md 文档) ----------------
ID_PUBLISHER = 0
ID_SUBSCRIBER = 1
ID_TIME = 10
ID_TX_STOP = 11

SYNC_REQ = bytes([0xff, 0xfe, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff])


def build_frame(topic_id: int, data: bytes) -> bytes:
    """组 rosserial 帧: ff fe len size_chk topic data chk"""
    size = len(data)
    size_chk = (0xff - ((size & 0xff) + ((size >> 8) & 0xff))) & 0xff
    chk = (topic_id & 0xff) + (topic_id >> 8) + sum(data)
    return bytes([
        0xff, 0xfe,
        size & 0xff, (size >> 8) & 0xff, size_chk,
        topic_id & 0xff, (topic_id >> 8) & 0xff,
    ]) + data + bytes([(0xff - (chk % 256)) & 0xff])


class FrameParser:
    """rosserial 拆帧器 (带损坏帧防御)"""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)
        frames = []
        while True:
            if len(self.buf) < 2:
                break
            if self.buf[0] != 0xff or self.buf[1] != 0xfe:
                self.buf.pop(0)
                continue
            if len(self.buf) < 5:
                break
            size = self.buf[2] | (self.buf[3] << 8)
            if ((self.buf[2] + self.buf[3] + self.buf[4]) & 0xff) != 0xff:
                self.buf.pop(0)
                continue
            if size > 512:  # 损坏帧防御
                self.buf.pop(0)
                continue
            if len(self.buf) < 7 + size + 1:
                break
            topic = self.buf[5] | (self.buf[6] << 8)
            data = bytes(self.buf[7:7 + size])
            chk = self.buf[7 + size]
            del self.buf[:7 + size + 1]
            if ((topic & 0xff) + (topic >> 8) + sum(data) + chk) & 0xff == 0xff:
                frames.append((topic, data))
        return frames


def _read_string(data, off):
    l = struct.unpack('<I', data[off:off + 4])[0]
    return data[off + 4:off + 4 + l].decode(errors='replace'), off + 4 + l


def parse_topic_info(data):
    """本固件 TopicInfo: uint16 topic_id + 3×string + int32 buffer_size"""
    topic_id = struct.unpack('<H', data[0:2])[0]
    off = 2
    name, off = _read_string(data, off)
    mtype, off = _read_string(data, off)
    md5, off = _read_string(data, off)
    return topic_id, name, mtype


def find_serial_by_vidpid(vid, pid):
    """按 USB 芯片型号找串口 (ttyUSB 编号不稳定)
    CH340(1a86:7523)=底盘, CP2102(10c4:ea60)=雷达"""
    if os.path.exists('/dev/rikibase') and vid == '1a86':
        return '/dev/rikibase'
    if os.path.exists('/dev/rikilidar') and vid == '10c4':
        return '/dev/rikilidar'
    # 方法1: pyserial list_ports (最可靠, 直接给 vid/pid)
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            if (p.vid is not None and p.pid is not None
                    and p.vid == int(vid, 16) and p.pid == int(pid, 16)):
                return p.device
    except Exception:
        pass
    # 方法2: sysfs 逐级上溯找 idVendor
    for node in glob.glob('/sys/class/tty/ttyUSB*'):
        d = os.path.realpath(node + '/device')
        while True:
            vf = os.path.join(d, 'idVendor')
            if os.path.exists(vf):
                try:
                    v = open(vf).read().strip()
                    p = open(os.path.join(d, 'idProduct')).read().strip()
                    if v == vid and p == pid:
                        return '/dev/' + os.path.basename(node)
                except Exception:
                    pass
                break
            par = os.path.dirname(d)
            if par == d:
                break
            d = par
    return None


# ---------------- 节点 ----------------
class RikibotBaseNode(Node):
    def __init__(self):
        super().__init__('rikibot_base_node')

        # 参数
        self.declare_parameter('serial_port', 'auto')   # auto=按 CH340 自动找底盘
        self.declare_parameter('baud', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('cmd_timeout', 0.2)     # cmd_vel 停车看门狗 (s)
        self.declare_parameter('cmd_hz', 50.0)          # 指令下发频率
        self.declare_parameter('publish_tf', True)
        # 里程计比例校准 (落地标定后调整; 实测空载约 0.96~1.0)
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self._baud = self.get_parameter('baud').value
        port = self.get_parameter('serial_port').value
        if port == 'auto':
            auto = find_serial_by_vidpid('1a86', '7523')   # CH340 = 底盘
            if auto:
                port = auto
            else:
                self.get_logger().fatal(
                    '未找到底盘(CH340 1a86:7523)串口, 请检查 USB 连接')
                raise SystemExit(1)
        self._port = port
        baud = self._baud

        # 发布/订阅
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.imu_pub = self.create_publisher(Imu, 'imu_raw', 50)
        self.bat_pub = self.create_publisher(Float32, 'battery', 10)
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_cb, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # rosserial 连接 (超时短一点, 避免阻塞读拖慢 10ms 定时器)
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
        except serial.SerialException as e:
            self.get_logger().fatal(
                f'无法打开串口 {port} @ {baud}: {e}\n'
                '请检查: USB是否插入 / 端口是否正确 / 是否被其他进程占用 (lsof)')
            raise SystemExit(1)
        self.parser = FrameParser()
        self.announcements = []   # (通道, tid, name, type)
        self.vel_tid = None    # raw_vel 话题 id
        self.imu_tid = None    # raw_imu 话题 id
        self.bat_tid = None    # battery 话题 id
        self.cmd_tid = None    # cmd_vel 控制话题 id
        self._discover()
        self._assign_topics()
        if self.cmd_tid is None:
            self.get_logger().warn('未发现 cmd_vel 控制话题, 使用默认 101')
            self.cmd_tid = 101
        self.get_logger().info(
            f'话题: vel={self.vel_tid} imu={self.imu_tid} bat={self.bat_tid} cmd={self.cmd_tid}')

        # 状态
        self.x = self.y = self.th = 0.0
        self.vx = self.vy = self.wz = 0.0
        self.last_vel_time = None
        self.last_cmd_time = None
        self.cmd = Twist()          # 最近一次指令
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.get_parameter('odom_frame').value
        self.odom_msg.child_frame_id = self.get_parameter('base_frame').value
        self._set_covariance()
        # 诊断计数
        self.vel_frames = 0
        self.odom_pubs = 0
        self.time_syncs = 0
        self._last_vel_count = 0
        self._serial_err_logged = False

        # 定时器: 读串口 10ms + 指令看门狗 + rosserial 时间同步续约
        self.create_timer(0.01, self.read_serial_cb)
        self.create_timer(1.0 / self.get_parameter('cmd_hz').value, self.cmd_watchdog_cb)
        self.create_timer(5.0, self.keepalive_cb)

    # ---------- rosserial 握手: 与验证过的独立脚本一致 ----------
    def _discover(self, timeout=3.0):
        """发同步请求并解析; 一旦发现 cmd_vel 控制话题立即停止(不洪泛)"""
        self.ser.reset_input_buffer()
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.ser.write(SYNC_REQ)
            time.sleep(0.1)
            got = self.ser.read(self.ser.in_waiting or 0)
            if got:
                for topic, data in self.parser.feed(got):
                    if topic in (ID_PUBLISHER, ID_SUBSCRIBER):
                        tid, name, mtype = parse_topic_info(data)
                        self.announcements.append((topic, tid, name, mtype))
                        if (topic == ID_SUBSCRIBER and 'Twist' in mtype
                                and name == 'cmd_vel'):
                            return

    def _assign_topics(self):
        """从公告分配话题 id; cmd_vel 只认订阅通道(100~124), 其余兜底已知稳定 id"""
        for ch, tid, name, mtype in self.announcements:
            if 'Velocities' in mtype:
                self.vel_tid = tid
            elif 'Imu' in mtype:
                self.imu_tid = tid
            elif 'Battery' in mtype:
                self.bat_tid = tid
        # cmd_vel 控制: 仅订阅通道(SUBSCRIBER=1)且 id 100~124 才算
        cmds = [tid for ch, tid, n, t in self.announcements
                if ch == ID_SUBSCRIBER and 'Twist' in t and n == 'cmd_vel'
                and 100 <= tid < 125]
        if cmds:
            self.cmd_tid = cmds[0]
        # 兜底: 已知稳定 id (多次实机验证不变)
        if self.vel_tid is None:
            self.vel_tid = 125
        if self.imu_tid is None:
            self.imu_tid = 126
        if self.bat_tid is None:
            self.bat_tid = 127
        if self.cmd_tid is None:
            self.cmd_tid = 101

    # ---------- 读串口: 拆帧 -> 更新速度/传感器 ----------
    def read_serial_cb(self):
        try:
            # 与独立脚本一致: 无数据时阻塞等一个 timeout, 避免边界漏读
            data = self.ser.read(self.ser.in_waiting or 1)
        except Exception:
            return
        if not data:
            return
        for topic, payload in self.parser.feed(data):
            if topic == self.vel_tid and len(payload) >= 12:
                self._on_vel(payload[:12])
            elif topic == self.imu_tid and len(payload) >= 72:
                self._publish_imu(payload)
            elif topic == self.bat_tid and len(payload) >= 4:
                v = struct.unpack('<f', payload[:4])[0]
                msg = Float32()
                msg.data = v
                self.bat_pub.publish(msg)
            elif topic == ID_TIME:
                self._send_time_sync()

    def _send_time_sync(self):
        """应答底盘时间同步请求(否则固件 ~11s 后停止上报)"""
        self.time_syncs += 1
        try:
            t = self.get_clock().now().to_msg()
            data = struct.pack('<II', t.sec & 0xffffffff, t.nanosec & 0xffffffff)
            self.ser.write(build_frame(ID_TIME, data))
        except Exception:
            pass

    def keepalive_cb(self):
        """诊断 + 自适应恢复: 5s 没收到新数据才重发协商请求"""
        self.get_logger().info(
            f'[诊断] vel帧={self.vel_frames} odom发布={self.odom_pubs} '
            f'timesync={self.time_syncs} 缓冲={len(self.parser.buf)}B')
        if self.vel_frames == self._last_vel_count:
            try:
                self.ser.write(SYNC_REQ)   # 尝试重新协商恢复
            except Exception:
                pass
        self._last_vel_count = self.vel_frames

    def _on_vel(self, payload):
        """raw_vel: 3×float32 (vx,vy,wz) -> 梯形积分 odom + 发布"""
        self.vel_frames += 1
        vx, vy, wz = struct.unpack('<3f', payload)
        vx *= self.linear_scale
        vy *= self.linear_scale
        wz *= self.angular_scale
        now = self.get_clock().now()
        if self.last_vel_time is None:
            # 首帧: 只记录初值, 不积分
            self.last_vel_time = now
            self._pvx, self._pvy, self._pwz = vx, vy, wz
            self.vx, self.vy, self.wz = vx, vy, wz
            return
        dt = (now - self.last_vel_time).nanoseconds / 1e9
        self.last_vel_time = now
        if dt <= 0 or dt > 0.5:   # 丢弃异常间隔
            self._pvx, self._pvy, self._pwz = vx, vy, wz
            return
        # 梯形积分: 用 (上一帧+当前帧)/2 减小离散误差 (与官方驱动一致)
        dvx = (self._pvx + vx) / 2.0
        dvy = (self._pvy + vy) / 2.0
        dwz = (self._pwz + wz) / 2.0
        self._pvx, self._pvy, self._pwz = vx, vy, wz
        self.vx, self.vy, self.wz = vx, vy, wz

        # 麦轮/全向 三通道积分
        self.th += dwz * dt
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))
        self.x += (dvx * math.cos(self.th) - dvy * math.sin(self.th)) * dt
        self.y += (dvx * math.sin(self.th) + dvy * math.cos(self.th)) * dt

        stamp = now.to_msg()
        self.odom_msg.header.stamp = stamp
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)
        self.odom_msg.pose.pose.orientation.z = qz
        self.odom_msg.pose.pose.orientation.w = qw
        self.odom_msg.twist.twist.linear.x = self.vx
        self.odom_msg.twist.twist.linear.y = self.vy
        self.odom_msg.twist.twist.angular.z = self.wz
        self.odom_pub.publish(self.odom_msg)
        self.odom_pubs += 1

        if self.get_parameter('publish_tf').value:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_msg.header.frame_id
            t.child_frame_id = self.odom_msg.child_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    def _publish_imu(self, payload):
        """riki_msgs/Imu: 3×Vector3(linear_acc, angular_velocity, magnetic)"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('base_frame').value
        ax, ay, az = struct.unpack('<3d', payload[0:24])
        gx, gy, gz = struct.unpack('<3d', payload[24:48])
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.orientation_covariance[0] = -1.0   # 无姿态
        self.imu_pub.publish(msg)

    def _set_covariance(self):
        # 给一个保守的里程计协方差(实际可后续用 EKF 融合 IMU 校准)
        p = 1e-3
        for i in (0, 7, 14, 21, 28, 35):
            if i in (0, 7, 35):
                self.odom_msg.pose.covariance[i] = p
        for i in (0, 7, 35):
            self.odom_msg.twist.covariance[i] = p

    # ---------- cmd_vel 转发 + 看门狗 ----------
    def cmd_cb(self, msg: Twist):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def cmd_watchdog_cb(self):
        """持续下发: 指令新鲜就发指令, 超时/未收到就发零速(停车)"""
        if self.cmd_tid is None:
            return
        vx = self.cmd.linear.x
        vy = self.cmd.linear.y
        wz = self.cmd.angular.z
        if self.last_cmd_time is not None:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            if age > self.get_parameter('cmd_timeout').value:
                vx = vy = wz = 0.0
        try:
            data = struct.pack('<6d', vx, vy, 0.0, 0.0, 0.0, wz)
            self.ser.write(build_frame(self.cmd_tid, data))
        except Exception as e:
            # USB 拔出/串口异常: 只记录, 不让节点崩溃
            if not self._serial_err_logged:
                self.get_logger().error(f'串口写失败: {e}')
                self._serial_err_logged = True

    def shutdown(self):
        try:
            self.ser.write(build_frame(self.cmd_tid or 101, struct.pack('<6d', 0, 0, 0, 0, 0, 0)))
            self.ser.write(build_frame(ID_TX_STOP, b'\x00'))
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RikibotBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
