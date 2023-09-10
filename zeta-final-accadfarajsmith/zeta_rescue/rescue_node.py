import argparse
import time

import rclpy
import rclpy.node
from rclpy.action.client import ActionClient
from rclpy.task import Future

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from jmu_ros2_util import map_utils, transformations

# The Final Project
from std_msgs.msg import Empty
from zeta_competition_interfaces.msg import Victim
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from transform_service_srv.srv import TransformPose
from geometry_msgs.msg import PointStamped
import math


class RescueNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('rescue_node')
        latching_qos = QoSProfile(depth=1,
                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

        self.create_subscription(Empty, '/report_requested', self.empty_callback, 10)

        self.create_subscription(PoseArray, '/aruco_poses', self.aruco_callback, 10)

        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        self.publisher = self.create_publisher(Victim, '/victim', 10)

        self.create_timer(0.1, self.timer_callback)

        self.cli = self.create_client(TransformPose, 'transform_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # Set up the request object (most fields will be re-used each time.)
        self.req = TransformPose.Request()
        self.future = None
        self.check_future = False
        self.image = None
        self.point_stamped = PointStamped()
        
        self.victim_List = []
        self.ID = 0

    def eq(self, first, second):
        x = math.sqrt((first.point.point.x - second.point.point.x)**2 + (first.point.point.y - second.point.point.y)**2)
        if x >= 0.4:
            return False
        return True

    def empty_callback(self, empty_msg):
        self.light_bulb = True
        #self.get_logger().info('I got pinged')
        for v in self.victim_List:
            #self.get_logger().info('Victim ID %r' % v.description)
            if int(v.description) > 20:
                self.publisher.publish(v)


    def aruco_callback(self, aruco_msg):

        self.req.source_pose.header.frame_id = aruco_msg.header.frame_id
        self.req.target_frame = "map"

        if self.future is None:
            #self.get_logger().info('Making service request...')
            #self.req.source_pose.header.stamp = self.get_clock().now().to_msg()
            for i in range(len(aruco_msg.poses)):
                self.req.source_pose.pose.position.x = aruco_msg.poses[i].position.x
                self.req.source_pose.pose.position.y = aruco_msg.poses[i].position.y
                self.req.source_pose.pose.position.z = aruco_msg.poses[i].position.z

                self.req.source_pose.pose.orientation.x = aruco_msg.poses[i].orientation.x
                self.req.source_pose.pose.orientation.y = aruco_msg.poses[i].orientation.y
                self.req.source_pose.pose.orientation.z = aruco_msg.poses[i].orientation.z
                self.req.source_pose.pose.orientation.w = aruco_msg.poses[i].orientation.w
                
                self.check_future = True
                self.future = self.cli.call_async(self.req)



        

    def camera_callback(self, camera_msg):
        self.image = camera_msg

    def timer_callback(self):
        if self.check_future and self.future.done():  # Currepoint.nt server request has completed.
            try:
                response = self.future.result()
            except Exception as e:
                self.get_logger().info('Service call failed %r' % (e,))
            else:
                if response.success:
                    #self.point_stamped.header = response.target_pose.header
                    victim = Victim()
                    victim.id = self.ID
                    victim.point.header = response.target_pose.header
                    victim.point.point.x = response.target_pose.pose.position.x
                    victim.point.point.y = response.target_pose.pose.position.y
                    victim.point.point.z = response.target_pose.pose.position.z
                    # if self.image is not None:
                    #     victim.image = self.image
                    # else:
                    #     victim.image = None
                    victim.description = "1"
                else:
                    self.get_logger().info(
                        'Service call succeeded, but transform was not available.')
                self.future = None
                self.check_future = False
            # else:
            #     self.get_logger().info('Waiting for service call to finish...')
            #self.get_logger().info('Victim List %r' % self.victim_List)
            check = True

            if len(self.victim_List) > 0:
                for v in self.victim_List:
                    if self.eq(v, victim):
                        v.description = str(int(v.description) + 1)
                        if int(v.description) == 25:
                            v.image = self.image
                        check = False

            if check:
                self.ID += 1
                self.victim_List.append(victim)





def main():
    print('Hi from zeta_rescue.')
    rclpy.init()

    node = RescueNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
