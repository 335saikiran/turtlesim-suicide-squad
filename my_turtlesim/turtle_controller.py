#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from functools import partial
import random

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from mark_interfaces.msg import Turtle
from mark_interfaces.msg import TurtleArray
from mark_interfaces.srv import CatchTurtle
from turtlesim.srv import Spawn, Kill, SetPen
from rclpy.qos import QoSProfile, DurabilityPolicy

qos_profile = QoSProfile(depth=10)
qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL 
 
class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.pose: Pose = None  
        self.catch_closest_turtle_first_ = True
        self.turtle_to_catch: Turtle = None
        self.master_turtle = Turtle()
        self.master_turtle.name = "turtle1"
        self.master_turtle.x = 5.5
        self.master_turtle.y = 5.5
        self.master_turtle.theta = 0.0
        self.frequency_timer_ = 0.1
        self.t_array = TurtleArray()
        self.t_array.turtles.append(self.master_turtle)
        self.counter_ = 0
        self.color_set = False

        self.turtle_pose_subscriber_ = self.create_subscription(Pose, f"{self.master_turtle.name}/pose", self.turtle_pose_callback, 10)
        self.turtle_spawn_pos_subscriber_ = self.create_subscription(TurtleArray, "spawn_pos", self.turtle_spawn_pos_callback, qos_profile)
        self.turtle_set_pen_client_ = self.create_client(SetPen, f"{self.master_turtle.name}/set_pen")
        self.turtle_kill_service_client_ = self.create_client(CatchTurtle, "catch_turtle")
        self.cmd_vel_publisher_ = self.create_publisher(Twist, f"{self.master_turtle.name}/cmd_vel", 10)
        self.cmd_vel_timer_ = self.create_timer(self.frequency_timer_, self.cmd_vel_timer_callback)

    #A callback to subscribe the pose of current master turtle
    def turtle_pose_callback(self, pose: Pose):
        self.pose = pose

    #A callback to subscribe to turtle array that is being published from the spawner
    def turtle_spawn_pos_callback(self, msg: TurtleArray):
        #A condition to add only new turtles
        for turtle in msg.turtles:
            # Check if the turtle is already in the t_array
            if not any(t.name == turtle.name for t in self.t_array.turtles):
                self.t_array.turtles.append(turtle)

        # Set the first turtle from array to catch if not already set
        remaining_turtles = [t for t in self.t_array.turtles if t.name != self.master_turtle.name]
        #A condition to check if the first master turtle is turtle1 and if there is atleast one turtle in the array
        if self.master_turtle.name == "turtle1" and len(msg.turtles) > 0:
            self.get_logger().info("Turtle1 is the master turtle")
            self.turtle_to_catch = remaining_turtles[0]
        #A condition if the master turtle is not turtle1 and there is atleast one turtle in the array
        elif len(remaining_turtles) > 0:
            self.swap_turtle_to_catch(self.t_array)
        #A condition if there are no turtles in the array
        else:
            self.turtle_to_catch = None
            self.get_logger().info("No more turtles to catch.")

    #A method to swap the turtle to catch with the next turtle in the array
    def swap_turtle_to_catch(self, t_array):
        remaining_turtles = [t for t in t_array.turtles if t.name != self.master_turtle.name]
        if len(remaining_turtles) > 0:
            self.turtle_to_catch = self.get_nearest_turtle(self.t_array.turtles)
            if self.turtle_to_catch:
                self.get_logger().info(f"next to catch is {self.turtle_to_catch.name}")
        else:
            self.turtle_to_catch = None
            self.get_logger().info("No more turtles to catch.")

    #A method to get the nearest turtle from the array of turtles
    def get_nearest_turtle(self, arr):
        # Pick the next turtle as the new target, if available
        closest_turtle = None
        closest_distance = 10000.0
        for turtle in arr:
            self.get_logger().info(f"Checking turtle {turtle.name} at position ({turtle.x}, {turtle.y})")
            if turtle.name != self.master_turtle.name:
                distance = self.get_distance(turtle.x, turtle.y, self.pose.x, self.pose.y)
                if distance < closest_distance:
                    closest_distance = distance
                    closest_turtle = turtle
                    self.get_logger().info(f"New closest turtle: {closest_turtle.name} at distance {closest_distance}")

        return closest_turtle
    
    def get_distance(self, turtle1_x, turtle1_y, turtle2_x, turtle2_y):
        return math.sqrt((turtle1_x - turtle2_x) ** 2 + (turtle1_y - turtle2_y) ** 2)
    
    # A timer callback to control the turtle's movement towards the target turtle
    def cmd_vel_timer_callback(self):
        if self.pose is None or self.turtle_to_catch is None:
            self.get_logger().warn("Turtle not yet spawned")
            return

        dist_x = self.turtle_to_catch.x - self.pose.x
        dist_y = self.turtle_to_catch.y - self.pose.y
        distance = self.get_distance(self.turtle_to_catch.x, self.turtle_to_catch.y, self.pose.x, self.pose.y)
        
        # Check if the pen color is set for the master turtle
        if not self.color_set:
            self.get_logger().info(f"Setting pen color for turtle {self.master_turtle.name}")
            self.set_pen()
            self.color_set = True
            self.get_logger().info(f"color_set set to True after setting pen{self.master_turtle.name}")

        cmd = Twist()
    
        # Calculate the distance to the target turtle and adjust speed and direction
        if distance > 0.2:
            cmd.linear.x = 4* distance
            goal_theta = math.atan2(dist_y, dist_x)
            diff_theta = goal_theta - self.pose.theta
            if diff_theta > math.pi:
                diff_theta -= 2 * math.pi
            elif diff_theta < -math.pi:
                diff_theta += 2 * math.pi
            cmd.angular.z = 6 * diff_theta
        # If the turtle is close enough to the target turtle, stop moving
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info(f"Master turtle {self.master_turtle.name} reached target {self.turtle_to_catch.name}, killing master and swapping.")
            
            # Kill the master turtle
            self.call_kill_service(self.master_turtle.name)
            self.remove_turtle_by_name(self.master_turtle.name)
            self.color_set = False
            self.get_logger().info(f"pen color reset for turtle {self.master_turtle.name}")
            # Promote the target to master
            self.swap_master_turtle(self.turtle_to_catch)
            self.get_logger().info(f"Swapped master turtle to {self.turtle_to_catch.name}-> {self.master_turtle.name}")
            self.swap_turtle_to_catch(self.t_array)
            
            # self.get_logger().info(f"will attempt to set pen for turtle {self.master_turtle.name}")
            # self.set_pen()
            # self.get_logger().info(f"pen color set for master turtle {self.master_turtle.name}")

        # Publish the command velocity to the master turtle
        self.cmd_vel_publisher_.publish(cmd)
    
    #A method to remove a turtle from the turtle array by name
    def remove_turtle_by_name(self, name):
        for t in self.t_array.turtles:
            if t.name == name:
                self.t_array.turtles.remove(t)
                break

    #A callback to destroy the current pose subscriber and cmd_vel pubslisher
    #This callback, after removing the killed turtle from the turtle array, swaps the master turtle with the next turtle in the array
    def swap_master_turtle(self, new_master: Turtle):
        # Unsubscribe from old pose, publisher and client
        self.destroy_subscription(self.turtle_pose_subscriber_)
        self.destroy_publisher(self.cmd_vel_publisher_)
        self.destroy_client(self.turtle_set_pen_client_)
        # Update master_turtle
        self.master_turtle = new_master
        
        # Subscribe to new master's pose and cmd_vel
        self.turtle_pose_subscriber_ = self.create_subscription(
            Pose, f"{self.master_turtle.name}/pose", self.turtle_pose_callback, 10)
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, f"{self.master_turtle.name}/cmd_vel", 10)
        self.turtle_set_pen_client_ = self.create_client(SetPen, f"{self.master_turtle.name}/set_pen")

    #A method to request for the custom kill service in Spawner node
    
    def call_kill_service(self, turtle_name):
        while not self.turtle_kill_service_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Server kill...")
        request = CatchTurtle.Request()
        request.name = turtle_name
        future = self.turtle_kill_service_client_.call_async(request)
        future.add_done_callback(partial(self.call_kill_service_callback, turtle_name=turtle_name))

    #A callback function to handle the response from the kill service
    def call_kill_service_callback(self, future, turtle_name):
        response = future.result()
        if response.success:
            self.get_logger().info(f"Turtle {turtle_name} killed successfully")
        else:
            self.get_logger().error(f"Failed to kill turtle {turtle_name}")
    
    #A method to set a request to the SetPen service
    def set_pen(self):
        self.get_logger().info("Inside set_pen method")
        while not self.turtle_set_pen_client_.wait_for_service():
            self.get_logger().info("Waiting for SetPen service...")

        request = SetPen.Request()
        request.r = int(random.uniform(0, 255))
        request.g = int(random.uniform(0, 255))
        request.b = int(random.uniform(0, 255))
        request.width = int(5)
        request.off = int(0)

        future = self.turtle_set_pen_client_.call_async(request)
        future.add_done_callback(partial(self.set_pen_callback, request = request))
    
    #A callback function to handle the response from the SetPen service
    def set_pen_callback(self, future, request: SetPen.Request):
        response: SetPen.Response = future.result()
        if response == "":
            self.get_logger().info("Set pen color")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()