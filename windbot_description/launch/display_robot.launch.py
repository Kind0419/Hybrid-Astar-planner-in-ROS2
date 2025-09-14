import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    #default_rviz_config_path=urdf_tutorial_path+'/config/rviz/display_model.rviz'

    # 获取 windbot_description 包的共享目录
    urdf_tutorial_path = get_package_share_directory('windbot_description')
    # 构建 URDF 文件的默认路径
    default_model_path = urdf_tutorial_path + '/urdf/frist_robot.urdf'
    # 声明一个启动参数 'model'，默认值为 URDF 文件的绝对路径
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='URDF的绝对路径'
    )

    # 读取 URDF 文件内容作为机器人描述参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]
            )
    )

    # 启动 robot_state_publisher 节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 启动 joint_state_publisher 节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    # 启动 rviz2 节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        #arguments=['-d', default_rviz_config_path]
    )

    # 返回一个 LaunchDescription 对象，包含所有要启动的节点和参数声明
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
