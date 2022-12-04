from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
   return LaunchDescription([
      DeclareLaunchArgument(
         'frequency', default_value='4',
         description='topic frequency'
      ),
      DeclareLaunchArgument(
            'enable_recording',
            default_value='True'
        ),
      Node(
         package="beginner_tutorials",
         executable="listener",
         name="listener",
         parameters=[
            {'turtlename': 'turtle2'}
         ]
      ),
      Node(
         package="beginner_tutorials",
         executable='talker',
         name='talker',
         arguments = ['talk','0','0','1','0','0','0',],
         parameters=[
            {'frequency': LaunchConfiguration('frequency')}
         ],
      ),
    # record all topic (chatter, service_node)
      ExecuteProcess(
            condition=IfCondition(
                PythonExpression([
                    LaunchConfiguration('enable_recording')
                ])
            ),
            cmd=[[
                'ros2 bag record -o bag_output ',
                '/chatter ',
                '/service_node',
            ]],
            shell=True
        ),
    
   ])