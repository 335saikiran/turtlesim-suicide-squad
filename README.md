#turtlesim-suicide-squad  
A turtlesim project using ROS2 Jazzy - where a master turtle follows a spawned turtle and gets killed after reaching the target turtle (spawned turtle). The spawned turtle is then promoted to master turtle and targets the next spawned turtle.

It would look like this preview:

**What do we need to run this?**
I've run this on:
1. Ubuntu 24.04 LTS - Noble Numbat
2. ROS2 Jazzy Jalisco

**Additional Requirements**
I've used some custom interfaces from my mark_interfaces package. I have two messages and one service that are imported in the code.

Make sure to add the executables as shown in the CMakeLists.txt and package.xml files for mark_intefaces and my_turtle_sim packages.

After that, Follow these steps:

1. Clone the repository using this command below
<pre>git clone //</pre>
2. Then compile the package using
<pre>colcon build --packages-select // --symlink-install</pre>
3.  Run the launch file
<pre>ros2 launch mark_bringup turtlesim_suicide_squad.xml</pre>








