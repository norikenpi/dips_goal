from typing import List
import rclpy
import math
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_system_msgs.msg import AutowareState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose, Point, Quaternion
from autoware_auto_vehicle_msgs.msg import VelocityReport

from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from rclpy.clock import ROSClock
import time

class MySubscriberNode(Node):
    def __init__(self) -> None:
        super().__init__("my_subscriber_node")
        

        # パブリッシャーの生成
        self.pub = self.create_publisher(
            PoseStamped,
            "/planning/mission_planning/goal",
            10,
        )
        

        # サブスクライバーの生成
        """
        self.sub = self.create_subscription(
            AutowareState, # メッセージ型
            "/autoware/state", # トピック名
            self.listener_callback,
            10,
        )
        """
        self.sub = self.create_subscription(
            VelocityReport, # メッセージ型
            "/vehicle/status/velocity_status", # トピック名
            self.listener_callback,
            10,
        )


        self.client = self.create_client(
            ChangeOperationMode, 
            '/api/operation_mode/change_to_autonomous'
        )

        self.clock = ROSClock()


        self.vel_prev = 0

        # Goalデータのインスタンスを作成

        """真横
        goal1 = create_pose(3795.9, 73761.5, 0.0, 0.0, 0.0, -0.974428, 0.224701)
        goal2 = create_pose(3788.03, 73754.2, 0.0, 0.0, 0.0, -0.971564, 0.236777)
        goal3 = create_pose(3777.71, 73752.4, 0.0, 0.0, 0.0, -0.968994, 0.247083)
        goal4 = create_pose(3766.85, 73743.8, 0.0, 0.0, 0.0, -0.951416, 0.307908)#長いから中継
        goal5 = create_pose(3759.69, 73739.5, 0.0, 0.0, 0.0, -0.972747, 0.231871)

"""
        #前ずらし(数字は小さくなる。)
        goal1 = create_pose(3794.12, 73760.6, 0.0, 0.0, 0.0, -0.921315, 0.388816)
        goal2 = create_pose(3786.2, 73754.0, 0.0, 0.0, 0.0, -0.999933, 0.0115893)
        goal3 = create_pose(3775.86, 73751.2, 0.0, 0.0, 0.0, -0.925523, 0.378692)
        goal4 = create_pose(3766.85, 73743.8, 0.0, 0.0, 0.0, -0.951416, 0.307908)#長いから中継
        goal5 = create_pose(3757.0, 73738.4, 0.0, 0.0, 0.0, -0.983933, 0.178537)
        
        
        
        

        # Goalデータのリストを作成
        goal6 = create_pose(3741.74, 73751.0, 0.0, 0.0, 0.0, 0.849793, 0.527117)#長いから中継
        goal7 = create_pose(3737.54, 73758.9, 0.0, 0.0, 0.0, 0.876594, 0.48123) #1コーナー
        goal8 = create_pose(3719.24, 73753.5, 0.0, 0.0, 0.0, -0.93476, 0.355279)#2コーナー
        goal9 = create_pose(3721.68, 73742.6, 0.0, 0.0, 0.0, -0.52186, 0.853031)#2コーナー先

        #3コーナー手前
        #バック切替
        final_goal = create_pose(3702.17919921875, 73743.1953125, 0.0, 0.0, 0.0, 0.8537888223989135, 0.5206194836410528)
        self.goal_list = [goal1, goal2, goal3, goal4, goal5, goal6, goal7, goal8, goal9, final_goal]  # 他のgoalデータも同様に追加
        self.pass_count = 0



    
    def send_service_request(self):
        #if not self.client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('Service not available')
        #    return
        time.sleep(5)
        request = ChangeOperationMode.Request()
        # ここで必要ならリクエストにデータを設定
        self.future = self.client.call_async(request)
    

    # サブスクライブ時に呼ばれる
    def listener_callback(self, msg: VelocityReport):
        velocity = msg.longitudinal_velocity
        if velocity < 1.0e-3:
            velocity = 0.0
            
        vel_prev = self.vel_prev
        self.vel_prev = velocity 
        print(velocity)

        if velocity == 0.0 and vel_prev != 0.0: #ゴールに到着したら
            now = self.clock.now()
            goal_data = PoseStamped()
            print(now.to_msg())
            goal_data.header.stamp = now.to_msg()
            goal_data.header.frame_id = "map"

            goal_data.pose = self.goal_list[self.pass_count]
            self.pass_count = self.pass_count + 1

            self.pub.publish(goal_data)
            self.send_service_request()
            print(goal_data)
    """
    def listener_callback(self, msg: AutowareState):
        state_value_prev = self.state_value_prev
        state_value = msg.state
        self.state_value_prev = state_value
        print(state_value)

        if state_value == 6 and state_value_prev == 5: #ゴールに到着したら
            now = self.clock.now()
            goal_data = PoseStamped()
            print(now.to_msg())
            goal_data.header.stamp = now.to_msg()
            goal_data.header.frame_id = "map"

            goal_data.pose = self.goal_list[self.pass_count]
            self.pass_count = self.pass_count + 1

            self.pub.publish(goal_data)
            self.send_service_request()
            print(goal_data)
    """

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriberNode()

    # ノード終了の待機
    rclpy.spin(node)

    # ノードの破棄
    node.destroy_node()

    # RCLのシャットダウン
    rclpy.shutdown()

def create_pose(x, y, z, qx, qy, qz, qw):

    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose




if __name__ == "__main__":
    main()

            

# cmd = AckermannControlCommand()
#now = Clock().now().nanoseconds
#num = int(str(now)[0])

# ひたすら前進するように
# cmd.longitudinal.speed = math.sin(num)
# cmd.longitudinal.acceleration = math.sin(num)
# cmd.longitudinal.speed = -1000000000.0
# cmd.longitudinal.acceleration = -1000000000.0

# cmdをパブリッシュ
#self.pub.publish(state_value)
"""
goal_data.pose.position.x = 3762.1142578125
goal_data.pose.position.y = 73740.984375
goal_data.pose.position.z = 0.0
goal_data.pose.orientation.x = 0.0
goal_data.pose.orientation.y = 0.0
goal_data.pose.orientation.z = -0.9993257713189098
goal_data.pose.orientation.w = 0.0367151573313547
"""
