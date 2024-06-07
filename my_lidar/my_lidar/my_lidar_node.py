<<<<<<< HEAD
=======

>>>>>>> 1f3998f37781348e4a6552111969e1cb3e949293
import math
import os
import sys
import rclpy
import rospkg
import time
import tf_transformations
from rclpy.node import Node   
from rclpy.executors import ExternalShutdownException    
<<<<<<< HEAD
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
=======
from geometry_msgs.msg import Twist, Pose, Point  # Twistメッセージ型をインポート
from nav_msgs.msg import Odometry                 # Odometryメッセージ型をインポート
from sensor_msgs.msg import LaserScan             # LaserScanメッセージ型をインポート
>>>>>>> 1f3998f37781348e4a6552111969e1cb3e949293
from tf_transformations import euler_from_quaternion 
from ament_index_python.packages import get_package_share_directory

from rclpy.qos import qos_profile_sensor_data, QoSProfile

<<<<<<< HEAD
class MyLidar(Node):
    def __init__(self):
        super().__init__('my_lidar_node')
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.x0, self.y0, self.yaw0 = 0.0, 0.0, 0.0
        self.vel = Twist()
        self.set_vel(0.0, 0.0)
        qos_policy = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                depth=5)
        self.sub = self.create_subscription(LaserScan, 'merged_scan', self.lidar_cb, qos_profile=qos_policy)
        self.scan = LaserScan()
        self.scan.ranges = [-99.9] * 360
        self.snum = 360
        self.state = "FORWARD"  # 初期状態は前進
        self.start_rotation_time = None

    def lidar_cb(self, msg):
        self.scan = msg
        self.snum = len(msg.ranges)
        for i in range(len(msg.ranges)):
            if msg.ranges[i] < msg.range_min or msg.ranges[i] > msg.range_max:
                pass
            else:
                self.scan.ranges[i] = msg.ranges[i]
    
    def my_lidar(self):
        steps = 0
        self.set_vel(0.0, 0.0)
        rclpy.spin_once(self)

        while rclpy.ok():
            print(f'step={steps}')
            nscan = float(self.snum)
            li_right = int( 90.0 * (nscan / 360.0))
            li_left  = int(270.0 * (nscan / 360.0))
            li_front = int(180.0 * (nscan / 360.0))
            li_back  = 0

            dist = 2.0  # 前進してOKの距離   

            if self.state == "FORWARD":
                if self.scan.ranges[li_front] > dist:
                    self.set_vel(1.0, 0.0)  # 前進
                else:
                    self.set_vel(0.0, 0.0)  # 停止
                    self.state = "STOP"
                    self.start_rotation_time = time.time()  # 回転開始時間を記録
            elif self.state == "STOP":
                self.state = "ROTATE"
            elif self.state == "ROTATE":
                # 回転して障害物がない方向を探す
                if time.time() - self.start_rotation_time < 2.0:
                    self.set_vel(0.0, 0.5)  # 回転
                else:
                    # 回転を終えて前方の障害物がなくなったか確認
                    if self.scan.ranges[li_front] > dist:
                        self.set_vel(0.0, 0.0)
                        self.state = "FORWARD"
                    else:
                        # まだ前方に障害物がある場合、回転を続ける
                        self.start_rotation_time = time.time()  # 再度回転開始時間を更新
                        self.set_vel(0.0, 0.5)
            
            rclpy.spin_once(self)
            print(f'r[FRONT]={self.scan.ranges[li_front]}')
            print(f'r[ LEFT]={self.scan.ranges[li_left]}')
            print(f'r[ BACK]={self.scan.ranges[li_back]}')
            print(f'r[RIGHT]={self.scan.ranges[li_right]}')

            time.sleep(0.1)
            steps += 1
    
    def get_pose(self, msg):
=======
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
>>>>>>> 1f3998f37781348e4a6552111969e1cb3e949293
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
<<<<<<< HEAD
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion((q_x, q_y, q_z, q_w))
        return x, y, yaw        
        
    def odom_cb(self, msg):
        self.x, self.y, self.yaw = self.get_pose(msg)
        self.get_logger().info(f'x={self.x:.2f} y={self.y:.2f}[m] yaw={self.yaw:.2f}[rad/s]')
    
    def set_vel(self, linear, angular):
        self.vel.linear.x = linear
        self.vel.angular.z = angular
    
    def move_distance(self, dist):
        error = 0.05
        diff = dist - math.sqrt((self.x - self.x0) ** 2 + (self.y - self.y0) ** 2)
        if math.fabs(diff) > error:
            self.set_vel(1.5, 0.0)
=======
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
>>>>>>> 1f3998f37781348e4a6552111969e1cb3e949293
            return False
        else:
            self.set_vel(0.0, 0.0)
            return True

<<<<<<< HEAD
    def rotate_angle(self, angle):
        error = 0.01
        diff = angle - (self.yaw - self.yaw0)
        if math.fabs(diff) > error:
            self.set_vel(0.0, 0.5 * math.copysign(1, diff))
            return False
        else:
            self.set_vel(0.0, 0.0)
            return True

    def timer_callback(self):
        self.pub.publish(self.vel)


def main():
=======
    def rotate_angle(self, angle):  # 指定した角度angleを回転する
        # このメソッドは間違っています．move_distanceを参考に完成させてください．
        self.set_vel(0.0, 0.25)
        return False

    def timer_callback(self):  # タイマーのコールバック関数
        self.pub.publish(self.vel)  # 速度指令メッセージのパブリッシュ 


def main():  # main関数
>>>>>>> 1f3998f37781348e4a6552111969e1cb3e949293
    rclpy.init()
    node = MyLidar()
    try:
        node.my_lidar()
    except KeyboardInterrupt:
<<<<<<< HEAD
        print('Ctrl+Cが押されました．')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
=======
        print('Ctrl+Cが押されました．')     
    finally:
        rclpy.shutdown()






>>>>>>> 1f3998f37781348e4a6552111969e1cb3e949293

