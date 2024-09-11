from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="control",
            executable="main",
            name="main",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"Kp": 4.525,
                "Ki": 1.0,
                "Kd": 2.0,
                "Mode": "pid",
                "Status": "OFF_"}
            ]
        )
    ])