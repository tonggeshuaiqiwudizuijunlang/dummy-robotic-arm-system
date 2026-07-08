import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import struct
import math
import binascii
import serial
import serial.tools.list_ports
import time

class DummyUSBSenderNode(Node):
    def __init__(self):
        super().__init__('dummy_usb_sender_test')
        
        # 1. 关节基础配置
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.latest_target_positions = [0.0] * 6
        self.received_angles = [0.0] * 6
        self.is_finished = 0
        
        # 2. ROS 通讯
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.publisher_feedback = self.create_publisher(JointState, '/joint_states_feedback', 10)
        self.publisher_deg = self.create_publisher(JointState, '/joint_states2', 10) # 角度制发布
        
        # 3. 串口初始化
        self.port = "/dev/ttyUSB1"
        self.baudrate = 115200
        self.ser = None
        self.buffer = bytearray()
        
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01) # 短超时
            self.get_logger().info(f"成功打开串口: {self.port}，波特率: {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"无法打开串口 {self.port}: {e}")

        # 4. 统一控制定时器 (50Hz / 20ms)
        self.timer_period = 0.02 
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info(f"50Hz 统一控制循环已启动 (发送 & 接收模式)")

    def listener_callback(self, msg):
        """仅更新最新的目标角度 (弧度制)"""
        try:
            positions = {}
            for i, name in enumerate(msg.name):
                positions[name] = msg.position[i]
            
            if all(j in positions for j in self.joint_names):
                self.latest_target_positions = [positions[j] for j in self.joint_names]
        except Exception as e:
            pass

    def control_loop(self):
        """50Hz 定时任务：发送 + 接收解析 + 目标发布"""
        
        # --- A. 目标角度发布到 /joint_states2 (角度制, 50Hz 持续) ---
        j_deg_now = [math.degrees(pos) for pos in self.latest_target_positions]
        deg_msg = JointState()
        deg_msg.header.stamp = self.get_clock().now().to_msg()
        deg_msg.name = self.joint_names
        deg_msg.position = j_deg_now
        self.publisher_deg.publish(deg_msg)

        if not self.ser or not self.ser.is_open:
            return

        # --- B. 发送流程 (0xAA) ---
        try:
            # 数据格式: uint8(0xAA) + 6*float(j_deg) + uint16(0xFFFB)
            packed_send = struct.pack('<B6fH', 0xAA, *j_deg_now, 0xFFFB)
            self.ser.write(packed_send)
        except Exception as e:
            self.get_logger().error(f"发送数据失败: {e}")

        # --- C. 接收解析 (0x5A) ---
        try:
            # 读取所有可用字节
            if self.ser.in_waiting > 0:
                self.buffer.extend(self.ser.read(self.ser.in_waiting))
            
            # 协议长度: header(1) + 6*float(24) + is_finished(1) + tailer(2) = 28 bytes
            packet_len = 28
            
            while len(self.buffer) >= packet_len:
                # 寻找包头 0x5A
                if self.buffer[0] != 0x5A:
                    self.buffer.pop(0)
                    continue
                
                # 检查包尾 0xFFFB (little endian: FB FF)
                if self.buffer[26:28] != b'\xFB\xFF':
                    self.buffer.pop(0)
                    continue
                
                # 提取并解析数据
                packet = self.buffer[:packet_len]
                data = struct.unpack('<B6fBH', packet)
                
                self.received_angles = list(data[1:7])
                self.is_finished = data[7]
                
                # 处理完后移除已解析的数据
                self.buffer = self.buffer[packet_len:]
                
                # 发布反馈话题 (反馈话题是弧度制)
                self.publish_feedback()
                
                # 打印日志
                status_str = "已完成" if self.is_finished else "移动中"
                self.get_logger().info(f"[反馈] 状态:{status_str} | J1:{self.received_angles[0]:.2f} J2:{self.received_angles[1]:.2f} J3:{self.received_angles[2]:.2f}")
                
        except Exception as e:
            self.get_logger().error(f"接收解析失败: {e}")

    def publish_feedback(self):
        """发布反馈消息到 /joint_states_feedback"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [math.radians(deg) for deg in self.received_angles] # 转回弧度发布
        self.publisher_feedback.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyUSBSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
