#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped  # 如果你的 topic 是 geometry_msgs/msg/Pose，請改成 Pose
import numpy as np
import time

class RobotStateSampler(Node):
    def __init__(self):
        super().__init__('robot_state_sampler')
        
        # 建立訂閱者 (請根據實際的訊息型態調整，例如 Pose 或 PoseStamped)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )
        
        self.x_history = []
        self.y_history = []
        self.duration = 60.0  # 監聽 60 秒
        
        self.get_logger().info(f'開始監聽 /robot_pose，預計收集 {self.duration} 秒的數據...')
        
        # 建立定時器，在 60 秒後觸發計算並關閉
        self.timer = self.create_timer(self.duration, self.calculate_statistics)
        self.start_time = time.time()

    def pose_callback(self, msg):
        # 提取 x, y 座標 (如果是 Pose 型態，請改成 msg.position.x)
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        self.x_history.append(x)
        self.y_history.append(y)

    def calculate_statistics(self):
        self.get_logger().info('時間到！正在計算統計數據...')
        
        if not self.x_history or not self.y_history:
            self.get_logger().error('錯誤：在 60 秒內沒有接收到任何 /robot_pose 資料！')
            rclpy.shutdown()
            return

        # 轉換為 numpy array 進行運算
        x_arr = np.array(self.x_history)
        y_arr = np.array(self.y_history)
        
        # 計算統計值
        x_mean, y_mean = np.mean(x_arr), np.mean(y_arr)
        x_std, y_std = np.std(x_arr), np.std(y_arr)
        x_range = np.max(x_arr) - np.min(x_arr)
        y_range = np.max(y_arr) - np.min(y_arr)
        
        # 印出美化後的結果
        print("\n" + "="*40)
        print(f" 📊 機器人定位軌跡統計結果 (共收集 {len(x_arr)} 筆資料)")
        print("="*40)
        print(f"【X 軸統計】")
        print(f"  - 平均值 (Mean):    {x_mean:.6f} m")
        print(f"  - 標準差 (Std Dev): {x_std:.6f} m")
        print(f"  - 範圍 (Range):     {x_range:.6f} m  (Min: {np.min(x_arr):.4f} ~ Max: {np.max(x_arr):.4f})")
        print("-"*40)
        print(f"【Y 軸統計】")
        print(f"  - 平均值 (Mean):    {y_mean:.6f} m")
        print(f"  - 標準差 (Std Dev): {y_std:.6f} m")
        print(f"  - 範圍 (Range):     {y_range:.6f} m  (Min: {np.min(y_arr):.4f} ~ Max: {np.max(y_arr):.4f})")
        print("="*40 + "\n")
        
        # 關閉節點
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateSampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('使用者手動終止監聽。')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()