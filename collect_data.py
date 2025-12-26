import signal
import threading
from collections import deque
from time import sleep
import open3d as o3d
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import sys
import cupy as cp
import yaml
sys.path.append("./Camera")
from HKCam import *
import cv2
# 定义一个Pcd类，用于存储点云数据
class Pcd:
    def __init__(self, pcd_name=""):
        self.pcd = o3d.geometry.PointCloud()
        self.pcd_name = pcd_name

    # 将本Pcd的pcd属性设置为传入的pcd，修改引用
    def set_pcd(self, pcd):
        self.pcd = pcd

    # 更新pcd属性的points
    def update_pcd_points(self, pc):
        self.pcd.points = o3d.utility.Vector3dVector(pc)

    #


class PcdQueue(object):
    def __init__(self, max_size=90, voxel_size=0.05):
        self.max_size = max_size  # 传入最大次数
        self.queue = deque(maxlen=max_size)  # 存放的是pc类型
        # 创建一个空的voxel
        # self.voxel = o3d.geometry.VoxelGrid()
        # self.voxel_size = voxel_size
        # self.pcd_all = o3d.geometry.PointCloud()
        self.pc_all = []  # 用于存储所有点云的numpy数组
        # 内部属性
        self.record_times = 0  # 已存点云的次数
        self.point_num = 0  # 已存点的数量

    # 添加一个点云
    def add(self, pc):  # 传入的是pc
        self.queue.append(pc)  # 传入的是点云，一份点云占deque的一个位置
        # print("append")
        if self.record_times < self.max_size:
            self.record_times += 1

        self.update_pc_all()
        self.point_num = self.cal_point_num()

    def get_all_pc(self):
        return self.pc_all

    # 把每个queue里的pc:[[x1,y1,z1],[x2,y2,x2],...]的点云合并到一个numpy数组中
    def update_pc_all(self):

        self.pc_all = np.vstack(self.queue)

    def is_full(self):
        return self.record_times == self.max_size

    # 获得队列中点的数量，而非队列的大小
    def cal_point_num(self):
        return len(self.pc_all)


class Lidar:
    def __init__(self) -> None:
        # 标志位
        self.flag = False  # 激光雷达接收启动标志
        self.init_flag = False  # 激光雷达接收线程初始化标志
        self.working_flag = False  # 激光雷达接收线程启动标志
        self.threading = None  # 激光雷达接收子线程
        self.stop_event = threading.Event()  # 线程停止事件

        self.min_distance = 0.1
        self.max_distance = 40
        self.lidar_topic_name = "/livox/lidar"  # 激光雷达话题名

        # 点云队列
        self.pcdQueue = PcdQueue(
            max_size=1000
        )  # 将激光雷达接收的点云存入点云队列中，读写上锁？
        # 激光雷达线程
        self.lock = threading.Lock()  # 线程锁
        if not self.init_flag:
            # 当雷达还未有一个对象时，初始化接收节点
            self.listener_begin(self.lidar_topic_name)
            # print("listener_begin")
            self.init_flag = True
            self.threading = threading.Thread(target=self.main_loop, daemon=True)

            # 线程启动

    def start(self):
        """
        开始子线程，即开始spin
        """
        if not self.working_flag:
            self.working_flag = True
            self.threading.start()

            # print("start@")
        # 获取所有点云

    def get_all_pc(self):
        with self.lock:
            return self.pcdQueue.get_all_pc()

    # 线程关闭
    def stop(self):
        """
        结束子线程
        """
        if (
            self.working_flag and self.threading is not None
        ):  # 关闭时写错了之前，写成了if not self.working_flag
            self.stop_event.set()
            rclpy.shutdown()
            self.working_flag = False
            print("stop")

    # 节点启动
    def listener_begin(self, laser_node_name="/livox/lidar"):
        rclpy.init()
        # 订阅话题
        self.node = Node("lidar_listener")
        self.node.create_subscription(
            PointCloud2, laser_node_name, self.listener_callback, 10
        )

    # 订阅节点子线程
    def main_loop(self):
        # 通过将spin放入子线程来防止其对主线程的阻塞
        rclpy.spin(self.node)

    def listener_callback(self, msg):

        if self.stop_event.is_set():
            print("stop is set")
            return
        if self.working_flag:
            # print("working")
            # 从消息中提取点云数据
            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            # 将点云数据转换为numpy数组
            points = np.array(
                list(points),
                dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
            )
            # print(len(points))
            # 将结构化数组转换为普通的float64数组
            points = np.stack([points["x"], points["y"], points["z"]], axis=-1).astype(
                np.float64
            )
            # 过滤点云
            dist = np.linalg.norm(points, axis=1)  # 计算点云距离

            points = points[dist > self.min_distance]  # 雷达近距离滤除
            # 第二次需要重新计算距离，否则会出现维度不匹配
            dist = np.linalg.norm(points, axis=1)  # 计算点云距离
            points = points[dist < self.max_distance]  # 雷达远距离滤除

            with self.lock:
                # 存入点云队列
                self.pcdQueue.add(points)

# 从numpy转到cupy
def np2cp(np_array):
        return cp.array(np_array)
# 从cupy转到numpy
def cp2np(cp_array):
        return cp.asnumpy(cp_array)


def show_pcd_info(pc):
        x = pc[:, 0]
        y = pc[:, 1]
        z = pc[:, 2]
        print('x: ', x.min(), x.max())
        print('y: ', y.min(), y.max())
        print('z: ', z.min(), z.max())

def load_camera_parameters(file_path):
    global camera_matrix, distortion_coefficients
    
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
        
        camera_matrix = np.array(data['camera_matrix']['data']).reshape(3,3)
        distortion_coefficients = np.array(data['distortion_coefficients']['data'])

# 调用函数读取参数
load_camera_parameters('./parameters/ost.yaml')


if __name__ == "__main__":
    lidar = Lidar()
    lidar.start()
    print("Collecting Data ...")
    print('camera_matrix:',camera_matrix)
    print('distortion_coefficients:',distortion_coefficients)
    cam = HKCam(0)
    frame = cam.getFrame()
    frame = cv2.undistort(frame, camera_matrix, distortion_coefficients)
    cv2.imwrite("img.png", frame)
    # cv2.waitKey(0)
    print("Gathering PCDS...")
    
    # 保存10s点云
    sleep(10)

    pc = lidar.get_all_pc()
    show_pcd_info(pc)
    np.savetxt("data/pcds.txt", pc)
    print('Done!')
    
    lidar.stop()
    
    
    