#!/usr/bin/env python3
import os
import rclpy
import xacro
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spawn_entity')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    robot_name = os.environ["YARP_ROBOT_NAME"]

    xacro_file = os.path.join(get_package_share_directory('xcub_'+robot_name+'_moveit_config'), 'config', robot_name+'.urdf.xacro')
    assert os.path.exists(xacro_file)
    robot_desc = xacro.process_file(xacro_file)
    robot_model = robot_desc.toprettyxml()

    req = SpawnEntity.Request()
    req.xml = robot_model
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

