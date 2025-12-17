#!/usr/bin/env python3
import random
import math
import rclpy
from rclpy.node import Node

from functools import partial
from turtlesim.srv import Spawn

class SpawnTurtleNode(Node) : 
    def __init__(self):
        super().__init__("spawn_turtle_node")
        self.name = "turtle"
        self.counter = 1
        self.timer_ = self.create_timer(1.0,self.spawn_turtle)



    def spawn_turtle(self):
        self.counter += 1
        turtle_name = self.name + str(self.counter)
        x = random.uniform(0.0,11.0)
        y = random.uniform(0.0,11.0)
        theta = random.uniform(0.0, 2*math.pi)
        self.call_spawn_turtle_server( x, y, theta, turtle_name)

        pass


    def call_spawn_turtle_server(self, x, y, theta, turtle_name):
        client_ = self.create_client(Spawn,"/spawn")
        while not client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server - [Spawn Turtles]")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn_turtle, x=x,y=y, theta=theta, turtle_name = turtle_name))

    def callback_call_spawn_turtle(self, future,x,y,theta,turtle_name):
        try:
            response = future.result()
            if response.name != "": 
                self.get_logger().info("Turtle: " + response.name + " is created!")

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args= None):
    rclpy.init(args=args)
    node = SpawnTurtleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
