import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess

# odometría
def generate_launch_description():
	odometry_node = Node(
			package = 'odometry',
			executable = 'odometry',
			output = 'screen'
	)

	teleop_node = ExecuteProcess(
		cmd=['gnome-terminal','--','ros2','run','teleop_twist_keyboard','teleop_twist_keyboard'],
	    output= 'screen'
	)

	ros_bag_node = ExecuteProcess(
		cmd=['ros2','bag','record','/PosicionX', '/PosicionY',],
	    output= 'screen'
		
	)
	

	micro_ros_node = ExecuteProcess(
        cmd=['gnome-terminal','--','ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyUSB0'],
        output='screen'
    )
 

	"""micro_ros_node = ExecuteProcess(
        cmd=['gnome-terminal','--','ssh', 'puzzlebot@10.42.0.1', 'ros2', 'run' ,'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyUSB0'],
        output='screen'
    )
"""


		
	l_d = LaunchDescription([odometry_node, teleop_node, ros_bag_node])
	return l_d