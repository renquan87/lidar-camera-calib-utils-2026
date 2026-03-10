import os
import shutil
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

calib_param = {
    "fExposureTime": 20000.0,
    'fGain': 15.0
}

# 标定文件保存目录（项目根目录下的 parameters/）
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
PARAM_DIR = os.path.join(PROJECT_ROOT, 'parameters')
BACKUP_DIR = os.path.join(PARAM_DIR, 'backups')
CALIB_FILE = os.path.join(PARAM_DIR, 'ost.yaml')


def backup_old_calibration():
    """将旧的标定文件备份到 backups/ 并以创建日期命名"""
    if not os.path.exists(CALIB_FILE):
        return
    os.makedirs(BACKUP_DIR, exist_ok=True)
    mtime = os.path.getmtime(CALIB_FILE)
    date_str = datetime.fromtimestamp(mtime).strftime('%Y%m%d_%H%M%S')
    backup_name = f'ost_{date_str}.yaml'
    backup_path = os.path.join(BACKUP_DIR, backup_name)
    if not os.path.exists(backup_path):
        shutil.copy2(CALIB_FILE, backup_path)
        print(f'[calib] 已备份旧标定文件 -> backups/{backup_name}')
    else:
        print(f'[calib] 备份已存在: backups/{backup_name}，跳过')


def generate_launch_description():
    # 启动前备份旧标定
    backup_old_calibration()

    camera_dir = get_package_share_directory('hnurm_camera')

    # camera_info_url 使用 file:// 协议指向项目 parameters 目录
    camera_info_url = f'file://{CALIB_FILE}'

    cam_node = Node(
        package='hnurm_camera',
        executable='hnurm_camera_node',
        parameters=[
            LaunchConfiguration('params_file'),
            calib_param,
            {'camera_info_url': camera_info_url}
        ]
    )

    # ChArUco 标定节点
    # ChArUco 板参数：12x9 格子，格子边长 30mm，ArUco 标记边长 22.5mm，字典 5x5_1000
    # 优势：对遮挡和反光更鲁棒，不需要检测到所有角点
    camera_calibration_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        parameters=[{'calibration_save_path': CALIB_FILE}],
        arguments=[
            '--pattern', 'charuco',
            '--size', '12x9',
            '--square', '0.03',
            '--charuco_marker_size', '0.0225',
            '--aruco_dict', '5x5_1000',
            '--no-service-check',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(camera_dir, 'params', 'default.yaml'),
            description='Full path to the ROS2 parameters file to use for the camera'
        ),
        cam_node,
        camera_calibration_node,
    ])
