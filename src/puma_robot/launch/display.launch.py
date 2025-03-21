
import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "puma_robot"
    urdf_file_name = "puma_3dof.urdf.xacro"
    rviz_config_file = "puma_config.rviz"

    urdf_file_path = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        urdf_file_name,
    )


    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        "rviz",
        rviz_config_file,
    )


    with open(urdf_file_path, "r") as f:
        robot_description_content = f.read()

    return launch.LaunchDescription([



        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description_content}]
        ),


        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
            output="screen"
        ),


        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path]
        )
    ])
