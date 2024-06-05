
import math
import os
import sys
import rclpy
import rospkg
import time
import tf_transformations
from rclpy.node import Node   
from rclpy.executors import ExternalShutdownException    
from geometry_msgs.msg import Twist, Pose, Point  # Twistメッセージ型をインポート
from nav_msgs.msg import Odometry                 # Odometryメッセージ型をインポート
from sensor_msgs.msg import LaserScan             # LaserScanメッセージ型をインポート
from tf_transformations import euler_from_quaternion 
from ament_index_python.packages import get_package_share_directory

from rclpy.qos import qos_profile_sensor_data, QoSProfile

class MyLidar(Node):  # 簡単なLiDARクラス
    def __init__(self):   # コンストラクタ
        super().__init__('my_lidar_node')   
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10) 
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0   
        self.x0, self.y0, self.yaw0 = 0.0, 0.0, 0.0
        self.vel = Twist()      # Twistメッセージ型インスタンスの生成
        self.set_vel(0.0, 0.0)  # 速度の初期化
        # LiDARを使うため追加
        qos_policy = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                depth = 5)
        self.sub = self.create_subscription(LaserScan, 'scan', self.lidar_cb, qos_profile=qos_policy)
        #self.sub = self.create_subscription(LaserScan, 'scan', self.lidar_cb, 1 )
        self.scan = LaserScan()  # LaserScanメッセージ型インスタンスの生成
        self.scan.ranges = [-99.9] * 360 # 取得したデータと区別するためありえない値で初期化
        self.snum = 360
        
    def lidar_cb(self, msg):  # LiDARのコールバック関数
        #print('kokokiteru')
        self.scan = msg
        
        print('range(len(msg.ranges))=' + str(len(msg.ranges)))
        self.snum = len(msg.ranges)
        #time.sleep(1)
        
        for i in range(len(msg.ranges)):
            if msg.ranges[i] < msg.range_min or msg.ranges[i] > msg.range_max:
                pass # 計測範囲外のデータは使わないような処理が必要．ここではパス．
            else:
                self.scan.ranges[i] = msg.ranges[i] 
    
    def my_lidar(self): 
        steps = 0
        self.set_vel(0.0, 0.0)      # 停止  
        rclpy.spin_once(self)       # コールバック関数をよび出す  

        while rclpy.ok():
            print(f'step={steps}')
            #Lidarの角度分解能にあわせて調整
            nscan = float(self.snum)
            li_right = int(270.0*(nscan/360.0))
            li_left  = int( 90.0*(nscan/360.0))
            li_front = 0
            li_back  = int(180.0*(nscan/360.0))

            
            dist = 0.5                      # 前進してOKの距離   
            if self.scan.ranges[li_front] > dist:  # 
                self.set_vel(0.2, 0.0)        # 前進 
            else:
                self.set_vel(0.0, 0.0)        # 停止
                
            rclpy.spin_once(self)
            #self.print_lidar_info() # scanトピックの値を表示するときはコメントアウト
            print(f'r[FRONT]={self.scan.ranges[li_front]}')     # 前
            print(f'r[ LEFT]={self.scan.ranges[li_left]}')    # 左
            print(f'r[ BACK]={self.scan.ranges[li_back]}')   # 後
            print(f'r[RIGHT]={self.scan.ranges[li_right]}')   # 右
  
            time.sleep(0.1)  # 0.1 [s]
            steps += 1
    
    def get_pose(self, msg):      # 姿勢を取得する
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(
            (q_x, q_y, q_z, q_w))
        return x, y, yaw        
        
    def odom_cb(self, msg):         # オドメトリのコールバック関数
        self.x, self.y, self.yaw = self.get_pose(msg)
        self.get_logger().info(
            f'x={self.x: .2f} y={self.y: .2f}[m] yaw={self.yaw: .2f}[rad/s]')     
    
    def set_vel(self, linear, angular):  # 速度を設定する
        self.vel.linear.x = linear   # [m/s]
        self.vel.angular.z = angular  # [rad/s]    
    
    def move_distance(self, dist):  # 指定した距離distを移動する
        error = 0.05  # 許容誤差 [m] 
        diff = dist - math.sqrt((self.x-self.x0)**2 + (self.y-self.y0)**2) 
        if math.fabs(diff) > error:
            self.set_vel(0.25, 0.0)
            return False
        else:
            self.set_vel(0.0, 0.0)
            return True

    def rotate_angle(self, angle):  # 指定した角度angleを回転する
        # このメソッドは間違っています．move_distanceを参考に完成させてください．
        self.set_vel(0.0, 0.25)
        return False

    def timer_callback(self):  # タイマーのコールバック関数
        self.pub.publish(self.vel)  # 速度指令メッセージのパブリッシュ 


def main():  # main関数
    rclpy.init()
    node = MyLidar()
    try:
        node.my_lidar()
    except KeyboardInterrupt:
        print('Ctrl+Cが押されました．')     
    finally:
        rclpy.shutdown()







