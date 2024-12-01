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

camera_matrix = np.array([[2359.228979, 0.000000, 706.732103],
                       [0.000000, 2355.309404, 554.259586],
                       [0.000000, 0.000000, 1.000000]])

extrinsic_matrix = np.array([
    [0.0169415, -0.999855, 0.00188374, -0.0467622],
    [-0.00618507, -0.00198877, -0.999979, 0.045783],
    [0.999837, 0.0169295, -0.00621787, -0.0447652],
    [0, 0, 0, 1]
])
R = np.array([[-0.2888,-0.5342,-0.7945],
         [-0.9214,-0.0703,-0.3822],
         [0.25999,-0.84243,-0.47193]])
T = np.array([1.9706,2.2637,2.87587])
intrinsic_matrix_T = np2cp(camera_matrix)
def show_pcd_info(pc):
        x = pc[:, 0]
        y = pc[:, 1]
        z = pc[:, 2]
        print('x: ', x.min(), x.max())
        print('y: ', y.min(), y.max())
        print('z: ', z.min(), z.max())
# 将雷达点云转换为相机图像平面深度图
def camera_to_image(pc): # 传入的是一个open3d的pcd格式的点云，返回的是一个n*3的矩阵，n是点云的数量，是np.array格式的
    # 相机坐标系下的点云批量乘以内参矩阵，得到图像坐标系下的u,v和z,类似于深度图的生成

    # 从numpy变为cupy
    pc = np2cp(pc)



    xyz = cp.dot(pc, intrinsic_matrix_T.T) # 得到的uvz是一个n*3的矩阵，n是点云的数量，是np.array格式的
    # np.savetxt("xyz.txt", cp2np(xyz))
    # 之前深度图没正确生成是因为没有提取z出来，导致原来的uv错误过大了
    # 要获得u,v,z，需要将xyz的第三列除以第三列
    uvz = cp.zeros(xyz.shape)
    uvz[:, 0] = xyz[:, 0] / xyz[:, 2]
    uvz[:, 1] = xyz[:, 1] / xyz[:, 2]
    uvz[:, 2] = xyz[:, 2]
    

    # 从cupy变为numpy
    uvz = cp2np(uvz)
    # np.savetxt("uvz.txt", uvz)
    
    return uvz


# 将生成的uvz转换为深度图
def generate_depth_map(pc): # 传入的pcd,返回的是一个深度图

    uvz = camera_to_image(pc) # 转换为uvz
    # 提取u,v,z
    u = uvz[:, 0].astype(int)
    v = uvz[:, 1].astype(int)
    z = uvz[:, 2]
    # 打深度值的范围
    print('Depth values range:', z.min(), z.max())

    print('x range', uvz[:, 0].min(), uvz[:, 0].max())
    print('y range', uvz[:, 1].min(), uvz[:, 1].max())
    print('z range', uvz[:, 2].min(), uvz[:, 2].max())
    # 按距离填充生成深度图，近距离覆盖远距离
    width, height = 1440 , 1080
    img_z = np.full((height, width), np.inf)
    valid = np.bitwise_and(np.bitwise_and((u >= 0), (u < width)),
                               np.bitwise_and((v >= 0), (v < height)))
    
    u, v, z = u[valid], v[valid], z[valid]
    z_scaler =256* (z - min(z)) / (max(z) - min(z))
    img_z = np.full((height,width), np.inf)
    for ui, vi, zi in zip(u, v, z_scaler):
        img_z[vi, ui] = min(img_z[vi, ui], zi)  # 近距离像素屏蔽远距离像素
    # 小洞和“透射”消除
    img_z_shift = np.array([img_z, \
                        np.roll(img_z, 1, axis=0), \
                        np.roll(img_z, -1, axis=0), \
                        np.roll(img_z, 1, axis=1), \
                        np.roll(img_z, -1, axis=1)])
    img_z = np.min(img_z_shift, axis=0)
#     # 将深度值填充到深度图中
#     for i in range(len(uvz)):
#         if valid[i]:
#             img_z[int(v[i]), int(u[i])] = min(img_z[int(v[i]), int(u[i])], z[i])
        
#     # 小洞和“透射”消除
#     img_z_shift = np.array([img_z, \
#                                 np.roll(img_z, 1, axis=0), \
#                                 np.roll(img_z, -1, axis=0), \
#                                 np.roll(img_z, 1, axis=1), \
#                                 np.roll(img_z, -1, axis=1)])
#     img_z = np.min(img_z_shift, axis=0) # img_z 是一个height*width的矩阵
#    # 转为可以显示的图像
#     img_z = np.where(img_z > 50 ,50, img_z)
#     img_z = cv2.normalize(img_z, None, 0, 200, cv2.NORM_MINMAX, cv2.CV_8U)
#     # img_z = cv2.normalize(img_z, None, 0, 200, cv2.NORM_MINMAX, cv2.CV_8U) # 远的看不到，就把最大值调小
#     img_z = cv2.applyColorMap(img_z, cv2.COLORMAP_COOL)
    return img_z

if __name__ == "__main__":
    cam = HKCam(0)
    frame = cam.getFrame()
    
    cv2.imwrite("ori.png", frame)
    # cv2.waitKey(0)
    lidar = Lidar()
    lidar.start()
    sleep(10)

    pc = lidar.get_all_pc()
    np.savetxt("org.txt", pc)
    distortion_matrix = np.array([-0.106887,0.098889,0.000438,-4.9e-05,0.0])
    # frame 去畸变
    frame = cv2.undistort(frame, camera_matrix, distortion_matrix)
    
    
    T = np2cp(T)  # Convert T to CuPy array
    extrinsic_matrix = np2cp(extrinsic_matrix)
    extrinsic_matrix_inv = cp.linalg.inv(extrinsic_matrix)
    # image = camera_to_image(pc)
    pc = np2cp(pc)
    # # Add a column of ones to the points
    pc = cp.hstack((pc, np.ones((pc.shape[0], 1))))
    # print(pc.shape)
    pc = cp.dot(pc, extrinsic_matrix_inv)
    # # 提取前三列
    pc = pc[:, :3]
    # # 从cupy变为numpy
    pc = cp2np(pc)
    # show_pcd_info(pc)
    np.savetxt("pc.txt", pc)
    # print(pc)
    image = generate_depth_map(pc)
    # cv2.imshow("frame", image)
    # cv2.waitKey(0)
    
    cv2.imwrite("frame.png", image)
    # print(len(pc))
    # # 保存点云
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(pc)
    # o3d.io.write_point_cloud("test.pcd", pcd)

    lidar.stop()
    