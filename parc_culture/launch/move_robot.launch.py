

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import  ParameterFile
from nav2_common.launch import RewrittenYaml
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource



parc_culture = FindPackageShare(package='parc_culture').find("parc_culture")
tb3_gazebo = FindPackageShare(package='tb3_gazebo').find("tb3_gazebo")

def generate_launch_description():
    ld = LaunchDescription()
    use_pose_manage = LaunchConfiguration("use_pose_manage", default=True)
    use_estimator   = LaunchConfiguration("use_estimator", default=True)
    option          = LaunchConfiguration("option", default="play")
    filename        = LaunchConfiguration("filename", default="test_bag")
    convert_topic   = LaunchConfiguration("convert_topic", default="odom_filtered")
    use_move_server = LaunchConfiguration("use_move_server", default=True)
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
   
    # rviz setup file content, task2 configuration
    
    # params file for all node content their parameters
    params_file = os.path.join(parc_culture, "params/nodes_parameters.yaml")
    # start simulation test or real robot
    use_simulation = LaunchConfiguration("use_simulation", default=True)
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={},
            root_key="",
            convert_types=True,
        ),
        allow_substs=True,
    )

    
    nav2_node = GroupAction(
            
            actions=[
                   
                    Node(
                        package="move_base",
                        executable="convert_parc",
                        name="OdomToPose",
                        parameters=[{"odom_topic": convert_topic}],

                        ),
                    Node(
                        condition=IfCondition(PythonExpression([use_move_server])),
                        package="move_base",
                        executable="nav_action_server",
                        name="MovePoint",
                        parameters=[configured_params],

                        ),
                    Node(
                        condition=IfCondition(PythonExpression([use_estimator])),
                        package="move_base",
                        executable="distance.py",
                        ),
                    Node(
                        condition=IfCondition(PythonExpression([use_pose_manage])),
                        package="move_base",
                        executable="pose_manage.py",
                        parameters=[configured_params, {"filename": filename}, {"option": option}]
                        )
                    
            ]
    )
     

    ld.add_action(nav2_node)
    

    return ld
