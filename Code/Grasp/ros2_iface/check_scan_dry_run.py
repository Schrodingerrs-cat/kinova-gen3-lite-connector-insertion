#!/usr/bin/env python3
"""Call scan only after confirming dry_run. No parameters are changed."""
import rclpy
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger


def request(node, kind, name, body, timeout):
    client = node.create_client(kind, name)
    try:
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f'{name} unavailable')
        future = client.call_async(body)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f'{name} timed out; do not submit another request')
        return future.result()
    finally:
        node.destroy_client(client)


def main():
    rclpy.init()
    node = rclpy.create_node('check_scan_dry_run')
    try:
        query = GetParameters.Request()
        query.names = ['dry_run', 'allow_grasp']
        response = request(node, GetParameters, '/grasp_node/get_parameters', query, 5)
        if len(response.values) != 2 or any(v.type != 1 for v in response.values):
            raise RuntimeError('Missing Boolean gate parameters')
        if not response.values[0].bool_value or response.values[1].bool_value:
            raise RuntimeError('Requires dry_run=True and allow_grasp=False; no scan requested')
        print('Gates verified. Requesting dry-run scan...', flush=True)
        result = request(node, Trigger, '/grasp/scan', Trigger.Request(), 30)
        print(f'success: {result.success}\n{result.message}', flush=True)
        print('Stored-pose feasibility output is not whole-arm collision validation.', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
