import launch
import launch_ros
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.parameter_descriptions


def generate_launch_description():
    robot_name_in_model="windbot"
    urdf_tutorial_path=get_package_share_directory('windbot_description')
    default_model_path=urdf_tutorial_path+"/urdf/windbot/windbot.urdf.xacro"
    default_world_path=urdf_tutorial_path+"/world/custom_room.world"
    action_declare_arg_mode_path=launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_model_path),
        description='URDF的绝对路径'
    )
    robot_description=launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ',launch.substitutions.LaunchConfiguration('model')]
        ),value_type=str
    )
    robot_state_publisher_node=launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description}]
    )
    launch_gazebo=launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'),'/launch','/gazebo.launch.py']),
            launch_arguments=[('world',default_world_path),('verbose','true')]
    )
    spawn_entity_node=launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description',
        '-entity',robot_name_in_model,]
    )
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node
    ])