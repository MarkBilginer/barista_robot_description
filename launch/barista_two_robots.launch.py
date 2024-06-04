import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

import xacro


def generate_launch_description():

        ####### DATA INPUT ##########
    xacro_file = 'barista_multiple_robot_model.urdf.xacro'
    package_description = "barista_robot_description"

    # Paths
    pkg_barista_robot_description = get_package_share_directory(package_description)
    install_dir = get_package_prefix(package_description)

    print("Resolved package path for barista_robot_description: ", pkg_barista_robot_description)

     # Setting up Gazebo environment variables
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    gazebo_models_path = os.path.join(pkg_barista_robot_description, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=[get_package_share_directory('barista_robot_description'), 'gazebo/worlds/barista_robot_empty.world'],
        description= "barista_robot_empty_world",
    )
    # Define the launch arguments for the Gazebo launch file
    gazebo_launch_args = {
        'verbose': 'false',
        'pause': 'false',
        'world': LaunchConfiguration('world')
    }

    # Include the Gazebo launch file with the modified launch arguments
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments=gazebo_launch_args.items(),
    )

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(pkg_barista_robot_description, "xacro", xacro_file)

    robot_name_1 = "rick"
    robot_name_2 = "morty"

    # convert XACRO file into URDF

    robot_state_publisher_node1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node1',
        namespace=robot_name_1,
        output='screen',
        emulate_tty=True,
        parameters=[{'frame_prefix': robot_name_1 + '/', 'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name_1])}]
    )

    robot_state_publisher_node2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node2',
        namespace=robot_name_2,
        output='screen',
        emulate_tty=True,
        parameters=[{'frame_prefix': robot_name_2 + '/', 'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name_2])}]
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'rick_and_morty_vis.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    # Spawn Entity Node
    spawn_entity_node1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name="spawn_entity_node1",
        namespace=robot_name_1,
        arguments=['-entity', robot_name_1, '-x', '-1.5', '-y', '-1.5', '-z', '0.0', '-topic', 'robot_description'],
        output='screen'
    )

    spawn_entity_node2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name="spawn_entity_node2",
        namespace=robot_name_2,
        arguments=['-entity', robot_name_2, '-x', '1.5', '-y', '1.5', '-z', '0.0', '-topic', 'robot_description'],
        output='screen'
    )

    #static_robot_publisher1 = Node(
    #        package='tf2_ros',
    #        executable='static_transform_publisher',
    #        name='robot1_tf_publisher',
    #        output='screen',
    #        emulate_tty=True,
    #        arguments=['1', '0', '0', '0', '0', '0', 'world', robot_name_1 + '/odom']
    #    )

    #static_robot_publisher2 = Node(
    #        package='tf2_ros',
    #        executable='static_transform_publisher',
    #        name='robot2_tf_publisher',
    #        output='screen',
    #        emulate_tty=True,
    #        arguments=['2', '0', '0', '0', '0', '0', 'world', robot_name_2 + '/odom']
    #    )


    return LaunchDescription([
        world_file_arg,
        gazebo,
        robot_state_publisher_node1,
        robot_state_publisher_node2,
        rviz_node,
        spawn_entity_node1,
        spawn_entity_node2
    ])