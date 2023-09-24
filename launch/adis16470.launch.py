import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    # xacro, urdf, yaml, rvizファイルのパスを左辺値に記入する
    imu_urdf_file_path = os.path.join(
        get_package_share_directory("adi_driver"),
        "config",
        "urdf",
        "adis16470_breakout.urdf"
    )
    print(imu_urdf_file_path)

    imu_rviz_file_path = os.path.join(
        get_package_share_directory("adi_driver"),
        "config",
        "rviz",
        "imu.rviz"
    )

    imu_urdf_file = open(imu_urdf_file_path, "r")
    robot_description = imu_urdf_file.read()
    imu_urdf_file.close()

    # Launchの引数オブジェクトを宣言する
    with_filter_arg = DeclareLaunchArgument(name="with_filter", default_value="true")
    with_rviz_arg = DeclareLaunchArgument(name="with_rviz", default_value="true")
    with_plot_arg = DeclareLaunchArgument(name="with_plot", default_value="false")
    imu_device_arg = DeclareLaunchArgument(name="device", default_value="/dev/ttyACM0")
    frame_id_arg = DeclareLaunchArgument(name="frame_id", default_value="imu")
    burst_read_arg = DeclareLaunchArgument(name="burst_read", default_value="false")
    publish_temperature_arg = DeclareLaunchArgument(name="publish_temperature", default_value="false")
    rate_arg = DeclareLaunchArgument(name="rate", default_value="100.0")
    publish_tf_arg = DeclareLaunchArgument(name="publish_tf", default_value="true")
    publish_debug_topics_arg = DeclareLaunchArgument(name="publish_debug_topics", default_value="false")

    # Nodeのオブジェクトを宣言する
    # 起動するノードのオブジェクトの宣言
    robot_state_publisher_node = Node(
        package='robot_state_publisher',                                # パッケージの名前
        executable='robot_state_publisher',                             # 実行ファイルの名前
        name='robot_state_publisher',                                   # ノードの名前
        arguments=[imu_urdf_file_path],                                 # main関数の引数
        parameters=[{'robot_description': robot_description}],          # ROSのパラメータ
        output='screen'                                                 # ターミナルにLogを出力する
    )

    imu_node = Node(
        package="adi_driver",
        executable="adis16470_node",
        name="adis16470_node",
        namespace="imu",
        parameters=[
            {"device": LaunchConfiguration("device"),
             "frame_id": LaunchConfiguration("frame_id"),
             "burst_read": LaunchConfiguration("burst_read"),
             "publish_temperature": LaunchConfiguration("publish_temperature"),
             "rate": LaunchConfiguration("rate")}
        ],
        output="screen"
    )

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        parameters=[
            {"use_mag": TextSubstitution(text="false"),
             "publish_tf": LaunchConfiguration("publish_tf"),
             "publish_debug_topics": LaunchConfiguration("publish_debug_topics")}
        ],
        condition=IfCondition(LaunchConfiguration("with_filter"))
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", imu_rviz_file_path],
        condition=IfCondition(LaunchConfiguration("with_rviz"))
    )

    gyro_plot_node = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="gyro",
        namespace="plot",
        arguments=["/imu/data_raw/angular_velocity/x:y:z"],
        condition=IfCondition(LaunchConfiguration("with_plot"))
    )

    accl_plot_node = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="accl",
        namespace="plot",
        arguments=["/imu/data_raw/linear_accleration/x:y:z"],
        condition=IfCondition(LaunchConfiguration("with_plot"))
    )

    return LaunchDescription([
        with_filter_arg,
        with_rviz_arg,
        with_plot_arg,
        imu_device_arg,
        frame_id_arg,
        burst_read_arg,
        publish_temperature_arg,
        rate_arg,
        publish_tf_arg,
        publish_debug_topics_arg,
        robot_state_publisher_node,
        imu_node,
        imu_filter_node,
        rviz_node,
        gyro_plot_node,
        accl_plot_node
    ])
