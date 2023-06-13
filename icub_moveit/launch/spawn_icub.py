#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spawn_entity')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    urdfName = 'iCub_robot'
    urdf = os.path.join(get_package_share_directory('icub_moveit_config'), 'config', urdfName + '.urdf')
    assert os.path.exists(urdf)

    req = SpawnEntity.Request()
    req.name = "icub"
    req.xml = open(urdf, 'r').read()
    req.robot_namespace = ""
    req.reference_frame = "world"

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result ' + str(future.result().success) + " " + future.result().status_message)
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

