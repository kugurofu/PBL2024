import math
import os
import sys
import rclpy
import rospkg
import time
import tf_transformations
from rclpy.node import Node   
from rclpy.executors import ExternalShutdownException    
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion 
from ament_index_python.packages import get_package_share_directory

from rclpy.qos import qos_profile_sensor_data, QoSProfile

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
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
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
            return False
        else:
            self.set_vel(0.0, 0.0)
            return True

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
    rclpy.init()
    node = MyLidar()
    try:
        node.my_lidar()
    except KeyboardInterrupt:
        print('Ctrl+Cが押されました．')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

