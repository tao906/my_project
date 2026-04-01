#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray, GoalStatus
import math
import matplotlib.pyplot as plt
import numpy as np
import os
import time

class AutoNavEvaluator(Node):
    def __init__(self):
        super().__init__('auto_nav_evaluator')

        self.declare_parameter('odom_topic', '/odom')
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_sub_callback, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status_callback, 10)

        self.get_logger().info(f"评估节点就绪! 导航结束后将自动延迟2秒停止记录...")

        self.is_recording = False
        self.stop_requested = False # [新增] 请求停止标志
        self.stop_time = 0.0        # [新增] 记录停止请求的时间
        self.final_status_str = ""  # [新增] 保存最终状态字符串
        self.current_goal_id = None 
        
        self.reset_data()

    def reset_data(self):
        self.timestamps = []
        self.positions_x = []
        self.positions_y = []
        self.linear_velocities = []
        self.angular_velocities = []
        self.last_pose = None
        self.total_distance = 0.0
        self.total_turn_angle = 0.0
        self.start_pose_record = None 
        self.goal_pose_record = None  
        self.final_pose_record = None
        self.stop_requested = False

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def goal_sub_callback(self, msg):
        self.goal_pose_record = (msg.pose.position.x, msg.pose.position.y)

    def status_callback(self, msg):
        if not msg.status_list: return

        latest_status = msg.status_list[-1]
        status_code = latest_status.status
        goal_uuid = latest_status.goal_info.goal_id.uuid

        # 1. 开始记录
        if status_code == GoalStatus.STATUS_EXECUTING:
            if not self.is_recording and not self.stop_requested:
                self.get_logger().info(f">>> 导航启动! 开始记录...")
                self.reset_data()
                self.is_recording = True
                self.current_goal_id = goal_uuid
            elif self.current_goal_id != goal_uuid:
                # 如果是新任务ID，立即重置
                if not self.stop_requested:
                    self.get_logger().info(f">>> 任务切换! 重置数据...")
                    self.reset_data()
                    self.current_goal_id = goal_uuid

        # 2. 触发停止 (但不立即停止，而是进入冷却倒计时)
        elif status_code in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED]:
            if self.is_recording and not self.stop_requested:
                result_map = {4:"成功到达", 5:"任务取消", 6:"任务失败"}
                self.final_status_str = result_map.get(status_code, "未知结束")
                self.get_logger().info(f"<<< 收到结束信号 ({self.final_status_str})，继续录制 2秒 以捕捉刹停细节...")
                self.stop_requested = True
                self.stop_time = self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg):
        # 只有在录制状态下才处理
        if not self.is_recording: return

        current_time = self.get_clock().now().nanoseconds / 1e9

        # [新增] 检查是否到了停止时间
        if self.stop_requested:
            if (current_time - self.stop_time) > 2.0: # 延迟 2秒
                self.is_recording = False
                self.stop_requested = False
                self.generate_report(self.final_status_str)
                return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = msg.twist.twist.linear.x 
        w = msg.twist.twist.angular.z 
        yaw = self.get_yaw(msg.pose.pose.orientation)

        if self.start_pose_record is None: self.start_pose_record = (x, y)
        self.final_pose_record = (x, y)

        self.timestamps.append(current_time)
        self.positions_x.append(x)
        self.positions_y.append(y)
        self.linear_velocities.append(v)
        self.angular_velocities.append(w)

        if self.last_pose is not None:
            self.total_distance += math.hypot(x - self.last_pose['x'], y - self.last_pose['y'])
            delta_yaw = yaw - self.last_pose['yaw']
            while delta_yaw > math.pi: delta_yaw -= 2.0 * math.pi
            while delta_yaw < -math.pi: delta_yaw += 2.0 * math.pi
            self.total_turn_angle += abs(delta_yaw)

        self.last_pose = {'x': x, 'y': y, 'yaw': yaw}

    def generate_report(self, status_str):
        if len(self.timestamps) < 5: return

        t = np.array(self.timestamps)
        t_rel = t - t[0]
        v = np.array(self.linear_velocities)
        w = np.array(self.angular_velocities)
        
        # 截断数据: 只要到停止信号那一刻的统计，后面的2秒只是为了画图
        # 这里为了简单，我们还是统计全程，因为最后2秒速度应该是0，不影响平均值太大
        
        print("\n" + "="*60)
        print(f"               自动导航性能评估报告")
        print("="*60)
        print(f"【任务结果】: {status_str}")
        print(f"【核心指标】")
        print(f"  1. 实际里程: {self.total_distance:.4f} m")
        print(f"  2. 总耗时  : {t_rel[-1] - 2.0:.4f} s (扣除2s缓冲)") # 修正耗时显示
        print(f"  3. 平均线速: {np.mean(np.abs(v)):.4f} m/s")
        print(f"  4. 最大线速: {np.max(np.abs(v)):.4f} m/s")
        print(f"  5. 平均角速: {np.mean(np.abs(w)):.4f} rad/s")
        print(f"  6. 最大角速: {np.max(np.abs(w)):.4f} rad/s")
        print(f"  7. 累计转向: {self.total_turn_angle:.4f} rad")
        print(f"  8. 累计转向: {math.degrees(self.total_turn_angle):.4f} deg")
        print("="*60 + "\n")

        self.plot_graphs(t_rel, v, w)

    def plot_graphs(self, t, v, w):
        fig, axs = plt.subplots(1, 2, figsize=(12, 5))
        
        # 图1: 线速度
        axs[0].plot(t, v, color='green', linewidth=1.5, label='Linear Vel')
        # 画一条竖线标记任务结束时间点
        stop_line_time = t[-1] - 2.0
        # if stop_line_time > 0:
            # axs[0].axvline(x=stop_line_time, color='red', linestyle='--', label='Goal Reached')
        
        axs[0].set_title('Linear Velocity [m/s]')
        axs[0].set_xlabel('Time [s]')
        axs[0].legend()
        axs[0].grid(True)

        # 图2: 角速度
        axs[1].plot(t, w, color='purple', linewidth=1.0, alpha=0.8, label='Raw Angular Vel')
        # if stop_line_time > 0:
        #     axs[1].axvline(x=stop_line_time, color='red', linestyle='--')
            
        axs[1].set_title('Angular Velocity [rad/s]')
        axs[1].set_xlabel('Time [s]')
        axs[1].grid(True)

        plt.tight_layout()
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        filename = f'nav_report_{timestamp_str}.png'
        try:
            plt.savefig(filename)
            self.get_logger().info(f"图表已保存: {os.path.abspath(filename)}")
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = AutoNavEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()