

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



move_base = FindPackageShare(package='move_base').find("move_base")
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
    use_rviz     = LaunchConfiguration("use_rviz", default=False)
   
    # rviz setup file content, task2 configuration
    rviz_config_file = os.path.join(move_base, "rviz/task2.rviz")
    
    # params file for all node content their parameters
    params_file = os.path.join(move_base, "params/nodes_parameters.yaml")
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

    turtlebot3_sim  = GroupAction(
        condition=IfCondition(PythonExpression([use_simulation])),
        actions=[
            SetParameter("use_sim_time", use_sim_time),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                        os.path.join(tb3_gazebo, "launch/turtlebot3_world.launch.py")
                        )
                        
                    ),
        ]
    )
    nav2_node = GroupAction(
            
            actions=[
                    
                    Node(
                        package="rviz2",
                        condition=IfCondition(PythonExpression([use_rviz])),
                        executable="rviz2",
                        name="rviz2",
                        output="screen",
                        arguments=["-d", rviz_config_file],
                        ),
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
    ld.add_action(turtlebot3_sim)
    

    return ld
