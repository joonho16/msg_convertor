# msg_convertor/launch/manus2jointstates_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    indices_arg = DeclareLaunchArgument(
        'indices',
        default_value='0,1,2,3,4,5,6,7,8,9,10',
        description="Indices to include (e.g. '0,5,10')"
    )
    timer_period_arg = DeclareLaunchArgument(
        'timer_period',
        default_value='0.01',
        description='Loop period in seconds'
    )
    glove_topic_arg = DeclareLaunchArgument(
        'glove_topic',
        default_value='manus_glove_0',
        description='Input Manus glove topic'
    )
    out_topic_arg = DeclareLaunchArgument(
        'out_topic',
        default_value='action',
        description='Output JointState topic'
    )

    node = Node(
        package='msg_convertor',
        executable='manus2jointstates',
        name='manus2jointstates',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'indices':       LaunchConfiguration('indices'),
            'timer_period':  LaunchConfiguration('timer_period'),
            'glove_topic':   LaunchConfiguration('glove_topic'),
            'out_topic':     LaunchConfiguration('out_topic'),
        }]
    )

    return LaunchDescription([
        indices_arg, timer_period_arg, glove_topic_arg, out_topic_arg,
        node
    ])
