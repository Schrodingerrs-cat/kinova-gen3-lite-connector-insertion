#!/usr/bin/env python3
"""Read grasp gates directly, bypassing ROS CLI discovery. No motion commands."""
import rclpy
from rcl_interfaces.srv import GetParameters


def main():
    rclpy.init()
    node = rclpy.create_node('check_grasp_settings')
    try:
        client = node.create_client(GetParameters, '/grasp_node/get_parameters')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Parameter service unavailable')
        request = GetParameters.Request()
        request.names = ['dry_run', 'pregrasp_only', 'allow_grasp']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done():
            raise RuntimeError('Parameter request timed out')
        response = future.result()
        if response is None or len(response.values) != len(request.names):
            raise RuntimeError('Invalid parameter response')
        for name, value in zip(request.names, response.values):
            if value.type != 1:
                raise RuntimeError(f'{name}: missing or not a Boolean parameter')
            print(f'{name}: {value.bool_value}', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
