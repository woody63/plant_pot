#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import math
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from functools import partial

class ImageCapturer(Node):
    def __init__(self):
        super().__init__('image_capturer_node')
        self.bridge = CvBridge()

        # 1. 配置参数
        self.num_cameras = 5               # 相机数量
        self.target_groups = 36            # 总组数 (0..35)，覆盖 0~350 度
        self.angle_interval = math.radians(10.0)  # 间隔 10 度
        self.output_dir = 'captured_images'
        
        # 旋转速度: 2.0 度/秒 
        self.cmd_vel = 1.2 * math.pi / 180.0
        
        # 创建输出目录
        os.makedirs(self.output_dir, exist_ok=True)

        # 2. 状态变量
        self.group_count = 0   # 当前正在进行的组号 (初始为0)
        
        # 标志位：当前组的 5 个相机是否都已保存
        # 索引 0 对应 cam_01, 索引 4 对应 cam_05
        self.cameras_saved_this_group = [False] * self.num_cameras
        
        self.initial_saved_all = False     # 第 0 组(静止)是否全部完成
        self.capture_ready = False         # 是否到达了旋转触发点
        
        # 关节追踪变量
        self.joint_index = -1
        self.start_angle_set = False
        self.prev_raw_angle = 0.0
        self.travel = 0.0

        # 3. 通信接口
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 匹配相机的发布策略
            history=HistoryPolicy.KEEP_LAST,
            depth=10
)
        # 动态订阅 5 个相机话题
        for i in range(1, self.num_cameras + 1):
            topic_name = f'/robot1/cam_{i:02d}/image_raw'
            self.create_subscription(
                Image,
                topic_name,
                partial(self.image_callback, cam_id=i),
                qos_profile
            )
            self.get_logger().info(f'Subscribed to {topic_name}')

        # 订阅关节状态 (用于计算旋转角度)
        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_callback, 
            50
        )

        # 发布速度指令
        self.vel_pub = self.create_publisher(
            Float64MultiArray, 
            '/turntable_velocity_controller/commands', 
            10
        )
        
        self.get_logger().info("Node initialized. Waiting for images...")

    # 辅助逻辑
    def start_rotation(self):
        """发布速度指令，开始旋转"""
        msg = Float64MultiArray()
        msg.data = [self.cmd_vel]
        self.vel_pub.publish(msg)
        self.get_logger().info(f'>>> Rotation STARTED at {math.degrees(self.cmd_vel):.1f} deg/s <<<')

    def stop_rotation(self):
        """停止旋转"""
        msg = Float64MultiArray()
        msg.data = [0.0]
        self.vel_pub.publish(msg)
        self.get_logger().info('>>> Rotation STOPPED. All tasks finished. <<<')

    @staticmethod
    def angle_diff(a, b):
        """计算角度差 (处理 -pi 到 pi 的跳变)"""
        d = a - b
        while d <= -math.pi:
            d += 2.0 * math.pi
        while d > math.pi:
            d -= 2.0 * math.pi
        return d

    def _ensure_bgr(self, cv_img, encoding):
        """确保图片格式为 BGR 以便 OpenCV 保存"""
        if encoding == 'bgr8':
            return cv_img
        elif encoding == 'rgb8':
            return cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        # 简单的单通道转3通道
        if len(cv_img.shape) == 2:
            return cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        return cv_img

    def save_image(self, msg, group_id, cam_id):
        """保存图片的核心函数"""
        try:
            # 转换 ROS Image -> OpenCV Image
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            except Exception:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 颜色校正
            cv_img = self._ensure_bgr(cv_img, getattr(msg, 'encoding', 'bgr8'))

            # 生成文件名: group_000_cam_01.png
            filename = f'group_{group_id:03d}_cam_{cam_id:02d}.png'
            full_path = os.path.join(self.output_dir, filename)
            
            success = cv2.imwrite(full_path, cv_img)
            if success:
                self.get_logger().info(f'Saved: {filename}')
            else:
                self.get_logger().error(f'Failed to write: {filename}')
                
        except Exception as e:
            self.get_logger().error(f'Error saving image: {e}')

    # 回调逻辑
    def joint_callback(self, msg: JointState):
        """
        监听转盘角度。
        逻辑：计算累积转动量，当达到 10, 20, 30... 度时，设置 capture_ready = True
        """
        # 寻找转盘关节索引
        if self.joint_index == -1:
            if 'turntable_joint' in msg.name:
                self.joint_index = msg.name.index('turntable_joint')
            else:
                return

        current_angle = msg.position[self.joint_index]

        # 初始化起始角度
        if not self.start_angle_set:
            self.prev_raw_angle = current_angle
            self.travel = 0.0
            self.start_angle_set = True
            return

        # 只有当第0组保存完毕并开始旋转后，才计算角度行程
        if not self.initial_saved_all:
            self.prev_raw_angle = current_angle # 保持更新，但不累积行程
            return

        # 计算累积行程
        delta = self.angle_diff(current_angle, self.prev_raw_angle)
        self.prev_raw_angle = current_angle
        self.travel += delta

        # 判断是否到达下一组触发点
        # 当前已完成 group_count 组 
        # 下一组的目标是 group_count + 1 (即第1组)
        # 第1组需要在 10度时触发，第2组在 20度...
        next_target_group = self.group_count + 1
        
        if next_target_group < self.target_groups:
            target_angle = next_target_group * self.angle_interval
            
            # 如果累积角度超过了目标角度，并且还没触发过
            if abs(self.travel) >= target_angle:
                if not self.capture_ready:
                    self.get_logger().info(f'--- Angle reached {math.degrees(abs(self.travel)):.2f}°. Triggering Group {next_target_group} ---')
                    
                    # 触发拍照信号
                    self.capture_ready = True
                    # 重置本组保存状态
                    self.cameras_saved_this_group = [False] * self.num_cameras

    def image_callback(self, msg: Image, cam_id: int):
        """
        相机回调函数。
        cam_id: 1~5
        """
        idx = cam_id - 1  # 列表索引 0~4

        # 1: 初始静止抓拍 (第 0 组)
        if not self.initial_saved_all:
            # 如果这个相机还没存过第0张
            if not self.cameras_saved_this_group[idx]:
                self.save_image(msg, 0, cam_id)
                self.cameras_saved_this_group[idx] = True
            
            # 检查：是否 5 个相机都存好了？
            if all(self.cameras_saved_this_group):
                self.get_logger().info('Group 000 (Static) complete. Starting rotation...')
                self.initial_saved_all = True
                self.group_count = 0  # 确认当前完成了第0组
                
                # 重置状态给下一组使用
                self.cameras_saved_this_group = [False] * self.num_cameras
                self.capture_ready = False 
                
                # 开始转动
                self.start_rotation()
            return

        # 2: 旋转动态抓拍 (第 1 ~ 35 组)
        # 只有当 joint_callback 说到位了 (capture_ready=True) 才保存
        if self.capture_ready:
            # 当前要存的是 group_count + 1 组
            target_group_id = self.group_count + 1
            
            if target_group_id < self.target_groups:
                if not self.cameras_saved_this_group[idx]:
                    self.save_image(msg, target_group_id, cam_id)
                    self.cameras_saved_this_group[idx] = True
                
                # 检查：本组 5 张是否齐了？
                if all(self.cameras_saved_this_group):
                    self.group_count += 1
                    self.get_logger().info(f'Group {self.group_count:03d} complete.')
                    
                    # 关闭触发，等待下一个 10 度
                    self.capture_ready = False
                    
                    # 检查是否全部任务完成 (0..35共36组)
                    if self.group_count >= (self.target_groups - 1):
                        self.stop_rotation()

def main(args=None):
    rclpy.init(args=args)
    node = ImageCapturer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_rotation()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()