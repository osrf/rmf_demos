#!/usr/bin/env python3

# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import uuid
import time
import argparse

import rclpy
from rclpy.node import Node
from rmf_task_msgs.srv import SubmitTask
from rmf_task_msgs.msg import TaskType, Delivery, Loop, Clean

###############################################################################


class TaskRequester:

    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()

        loop_arg = parser.add_argument_group("loop task arguments")
        loop_arg.add_argument("--loop", action="store_true",
                              help='select loop task')
        loop_arg.add_argument('-s', '--start',
                              help='Start waypoint')
        loop_arg.add_argument('-f', '--finish',
                              help='Finish waypoint')
        loop_arg.add_argument('-n', '--loop_num', 
                              help='Number of loops to perform',
                              type=int, default=1)

        delivery_arg = parser.add_argument_group("delivery task arguments")
        delivery_arg.add_argument("--delivery", action="store_true",
                                  help='select delivery task',)
        delivery_arg.add_argument('-p','--pickup',
                                  help='Start waypoint')
        delivery_arg.add_argument('-pd', '--pickup_dispenser',
                                  help='Pickup dispenser name')
        delivery_arg.add_argument('-d', '--dropoff',
                                  help='Finish waypoint')
        delivery_arg.add_argument('-di', '--dropoff_ingestor',
                                  help='Dropoff ingestor name')

        clean_arg = parser.add_argument_group("clean task arguments")
        clean_arg.add_argument("--clean", action="store_true",
                               help='select clean task')
        clean_arg.add_argument('-cs', '--clean_start',
                               help='clean start waypoint')

        self.args = parser.parse_args(argv[1:])
        self.node = rclpy.create_node('task_requester')
        self.submit_task_srv = self.node.create_client(
            SubmitTask, '/submit_task')


    def generate_task_req_msg(self):
        req_msg = SubmitTask.Request()
        try:
            if self.args.loop:
                req_msg.description.task_type.type = TaskType.TYPE_LOOP
                loop = Loop()
                loop.num_loops = self.args.loop_num
                loop.start_name = self.args.start
                loop.finish_name = self.args.finish
                req_msg.description.loop = loop
            elif self.args.delivery:
                req_msg.description.task_type.type = TaskType.TYPE_DELIVERY
                delivery = Delivery()
                delivery.pickup_place_name = self.args.pickup
                delivery.pickup_dispenser = self.args.pickup_dispenser
                delivery.dropoff_ingestor = self.args.dropoff
                delivery.dropoff_place_name = self.args.dropoff_ingestor
                req_msg.description.delivery = delivery
            elif self.args.clean:
                req_msg.description.task_type.type = TaskType.TYPE_CLEAN
                clean = Clean()
                clean.start_waypoint = self.args.clean_start
                req_msg.description.clean = clean
            else:
                return None
        except:
            return None

        req_msg.description.start_time = self.node.get_clock().now().to_msg()
        print(f"\n This is the req_msg: {req_msg}\n")
        return req_msg

    def main(self):
        req_msg = self.generate_task_req_msg()
        if req_msg is None:
            self.node.get_logger().error('Invalid input arguments, pls check')
            rclpy.shutdown()
            return

        self.node.get_logger().info("Submitting Task Request")
        try:
            future = self.submit_task_srv.call_async(req_msg)
            rclpy.spin_until_future_complete(
                self.node, future, timeout_sec=0.5)
            response = future.result()
            if response is None:
                self.node.get_logger().error('/submit_task srv call failed')
            else:
                self.node.get_logger().info(
                    f'New Dispatch task_id [{response.task_id}]')
        except Exception as e:
            self.node.get_logger().error('Error! Submit Srv failed %r' % (e,))

        rclpy.shutdown()

###############################################################################


def main(argv=sys.argv):
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)

    loop_requester = TaskRequester(args_without_ros)
    loop_requester.main()


if __name__ == '__main__':
    main(sys.argv)
