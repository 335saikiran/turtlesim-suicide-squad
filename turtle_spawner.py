#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
import random

from turtlesim.srv import Spawn, Kill
from mark_interfaces.msg import Turtle, TurtleArray
from mark_interfaces.srv import CatchTurtle
from rclpy.qos import QoSProfile, DurabilityPolicy

qos_profile = QoSProfile(depth=10)
qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL 
 
class TurtleSpawnerNode(Node): 
    def __init__(self):
        super().__init__("turtle_spawner")
        self.turtle_array = []
        self.turtle_name_prefix_ = "turtle_"
        self.turtle_name_counter_ = 1

        self.spawn_turtle_client_ = self.create_client(Spawn, "/spawn")
        self.turtle_kill_client_ = self.create_client(Kill, "/kill")
        self.turtle_spawn_pos_publisher_ = self.create_publisher(TurtleArray, "spawn_pos", qos_profile)
        self.turtle_spawn_timer_ = self.create_timer(0.5, self.spawn_new_turtle)
        self.turtle_kill_server_ = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)
 
    # Send random input to spawn a new turtle a
    def spawn_new_turtle(self):
        self.turtle_name_counter_ += 1
        self.turtle_name = self.turtle_name_prefix_ + str(self.turtle_name_counter_)
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2*3.14)
        self.spawn_turtle_callback(self.turtle_name, x, y, theta)
   
    # Client request to spawn a turtle with the given name, x, y, and theta
    def spawn_turtle_callback(self, turtle_name, x, y, theta):
        while not self.spawn_turtle_client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for spawn service...")
        
        # Create a request instance to assign values to spawn a turtle
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name
        self.get_logger().info(f"Spawning turtle {turtle_name} at ({x}, {y}) with theta {theta}") 
        
        #Create a future to handle the asynchronous call to the spawn service
        future = self.spawn_turtle_client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn_service, request = request))

    # Callback where a response is received and the new turtle is added to the turtle array and publisher
    def callback_call_spawn_service(self, future, request: Spawn.Request):
        response: Spawn.Response = future.result()
        if response.name != "":
            self.get_logger().info("New alive turtle is: " + response.name)
            #Create a Turtle instance to publish spawned turtle's position
            turtle_msg = Turtle()
            turtle_msg.name = response.name
            turtle_msg.x = request.x
            turtle_msg.y = request.y
            turtle_msg.theta = request.theta
            self.turtle_array.append(turtle_msg)
            #A method defined below to publish the turtle details
            self.publish_alive_turtles()
        else:
            self.get_logger().error("Failed to spawn turtle")
    
    #A method to publish - used here for publishing turtle array
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.turtle_array
        self.turtle_spawn_pos_publisher_.publish(msg)

    #A kill client that sends a request to kill a turtle by name. This belongs to the callback catch turtle function
    def call_kill_service(self, turtle_name):
        while not self.turtle_kill_client_.wait_for_service():
            self.get_logger().warn("Waiting for Server Kill...")
        
        request = Kill.Request()
        request.name = turtle_name
        future = self.turtle_kill_client_.call_async(request)
        future.add_done_callback(partial(self.call_kill_service_callback, turtle_name = turtle_name))
    
    #A call back where the turtle array is modified when a request is sent and the requested turtle is removed from
    # the array
    def call_kill_service_callback(self, future, turtle_name):
        # A condition to kill turtle1 - a separate condition because turtle1 is spawned by default
        if turtle_name == "turtle1":
            self.get_logger().info("Killing turtle1")
        else:
            #A condition to delete turtles in the array whose name is requested to the kill service 
            for (i, turtle) in enumerate(self.turtle_array):
                if turtle.name == turtle_name:
                    del self.turtle_array[i]
                    self.publish_alive_turtles()
                    break
    
    #Created a custom service call back to accept a turtle name request to kill and to respond with a boolean value
    def callback_catch_turtle(self, request: CatchTurtle.Request, response: CatchTurtle.Response):
        self.call_kill_service(request.name)
        response.success = True
        return response
        
 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()
