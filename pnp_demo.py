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
# 从 camera_locator 包中导入 PointsPicker 和 Anchor 类
from camera_locator.point_picker import PointsPicker
from camera_locator.anchor import Anchor
import yaml

import cv2

camera_matrix = None
distortion_coefficients = None
def load_camera_parameters(file_path):
    global camera_matrix, distortion_coefficients, camera_matrix_inv
    
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
        
        camera_matrix = (np.array(data['camera_matrix']['data']).reshape(3,3))
        camera_matrix_inv = np.linalg.inv(camera_matrix)
        distortion_coefficients = (np.array(data['distortion_coefficients']['data']))

def show_pcd_info(pc):
        x = pc[:, 0]
        y = pc[:, 1]
        z = pc[:, 2]
        print('x: ', x.min(), x.max())
        print('y: ', y.min(), y.max())
        print('z: ', z.min(), z.max())
# 定义一个函数用于读取雷达到相机外参数 ./parameters/lidar_camera.txt
def load_lidar_camera_parameters(file_path):
        global lidar2camera
        with open(file_path, 'r') as file:
                data = file.readlines()
                lidar2camera = (np.array([list(map(float, line.split())) for line in data]).reshape(4, 4))


def pcd_lidar2camera(pc):

    # # Add a column of ones to the points
    pc = np.hstack((pc, np.ones((pc.shape[0], 1))))
    # 将雷达点云转换到相机坐标系下
    extrinsic_matrix_inv = np.linalg.inv(lidar2camera)
    pc = np.dot(pc, extrinsic_matrix_inv)
    pc = pc[:, :3]

    return pc
def generate_depth_map(pc): # 传入的pcd,返回的是一个包含深度信息的图像

    uvz = camera_to_image(pc) # 转换为uvz
    # 提取u,v,z
    u = uvz[:, 0].astype(int)
    v = uvz[:, 1].astype(int)
    z = uvz[:, 2]

    # 按距离填充生成深度图，近距离覆盖远距离

    valid = np.bitwise_and(np.bitwise_and((u >= 0), (u < width)),
                               np.bitwise_and((v >= 0), (v < height)))
    
    u, v, z = u[valid], v[valid], z[valid]
    uvz = np.array([u, v, z]).T
    # print(uvz.shape)
    return uvz

cur_mouse = [0, 0]

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        global cur_mouse
        cur_mouse = (x, y)


# 获取投影后落在深度图矩形框内的点云 , 并不是反向映射，而是直接提取落在矩形框内的点云
def get_points_in_box(pc, box): # 传入的为pcd格式点云，box是一个元组，包含了矩形框的左上角和右下角的坐标：(min_u, min_v, max_u, max_v)，返回的是一个n*3的矩阵，n是点云的数量，是np.array格式的
        # box是一个元组，包含了矩形框的左上角和右下角的坐标：(min_u, min_v, max_u, max_v)
        # 好像没有小孔成像的感觉，似乎并不是一个锥形
        min_u, min_v, max_u, max_v = box
        print("select pcd rect:" + str(box))
        # 提取像素坐标系下坐标
        uvz = camera_to_image(pc)

        # 提取u,v,z
        u = uvz[:, 0]
        v = uvz[:, 1]
        z = uvz[:, 2]
        # 创建一个mask，标记落在矩形框中的点云,因为bitwise_and每次只能操作两个数组，所以需要分开操作
        mask1 = np.bitwise_and(u >= min_u, u <= max_u)
        mask2 = np.bitwise_and(v >= min_v, v <= max_v)
        mask3 = np.bitwise_and(mask1, mask2)
        mask = np.bitwise_and(mask3, z <= 70) # 滤除超出最大深度的点云
        # 获得落在矩形框中的点云的点云的index,pcd.points才是要筛选的点云
        box_points = np.asarray(pc)[mask]


        return box_points # 返回的是一个n*3的矩阵，n是点云的数量，是np.array格式的



# 将相机坐标系下点云转换到相机成像平面
def camera_to_image(pc): # 传入的是一个open3d的pcd格式的点云，返回的是一个n*3的矩阵，n是点云的数量，是np.array格式的
    # 相机坐标系下的点云批量乘以内参矩阵，得到图像坐标系下的u,v和z,类似于深度图的生成


    xyz = np.dot(pc, camera_matrix.T) # 得到的uvz是一个n*3的矩阵，n是点云的数量，是np.array格式的
    # np.savetxt("xyz.txt", cp2np(xyz))
    # 之前深度图没正确生成是因为没有提取z出来，导致原来的uv错误过大了
    # 要获得u,v,z，需要将xyz的第三列除以第三列
    uvz = np.zeros(xyz.shape)
    uvz[:, 0] = xyz[:, 0] / xyz[:, 2]
    uvz[:, 1] = xyz[:, 1] / xyz[:, 2]
    uvz[:, 2] = xyz[:, 2]
    
    return uvz


# 对pcd做体素降采样
def voxel_down_sample(pcd):
        # 使用VoxelGrid滤波器进行体素降采样
        pcd = pcd.voxel_down_sample(voxel_size=0.035)
        return pcd

# 对pcd进行滤波
def filter(pcd):
        # 体素降采样
        pcd = voxel_down_sample(pcd)
        # 去除离群点和噪声点
        # pcd = self.remove_outliers(pcd)

        return pcd

# 对点云进行DBSCAN聚类
def cluster(pcd):
        # with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels_np = np.array(pcd.cluster_dbscan(eps=0.4, min_points=10,
                                                    print_progress=False))
        # 如果没有找到任何簇，返回一个空的点云和中心点
        if len(labels_np) == 0 or np.all(labels_np == -1):
            return np.array([]), np.array([0, 0, 0])

        # 将labels_np转为cupy数组
        labels = np.asarray(labels_np)
        pcd_points = np.asarray(pcd.points)

        # 计算每个簇的大小
        max_label = labels.max().item()
        cluster_sizes = np.array([np.sum(labels == i).item() for i in range(max_label + 1)])

        # 判空
        if len(cluster_sizes) == 0:
            return np.array([]), np.array([0, 0, 0])

        # 找到最大簇的索引
        max_cluster_idx = np.argmax(cluster_sizes)

        # 找到最大簇的所有点
        max_cluster_points = pcd_points[labels == max_cluster_idx]

        # 计算最大簇的中心点
        centroid = np.mean(max_cluster_points, axis=0)

        # 从cupy转为numpy
        return max_cluster_points, centroid

# 提取open3d的pcd格式的点云的points，转为np.array格式
def get_points(pcd):
        pc = np.asarray(pcd.points)
        return pc

# 传入一个点云，选取距离的中值点
def get_center_mid_distance(pcd):
        pc = get_points(pcd)



        # 判空
        if len(pc) == 0:
            return [0,0,0]


        # 计算每个点的距离
        distances = np.linalg.norm(pc, axis=1)
        # 找到中值点的索引
        center_idx = np.argsort(distances)[len(distances) // 2]
        center = pc[center_idx]

        return center


# 传入相机坐标系的点云[x,y,z]，返回赛场坐标系的点云[x,y,z]，TODO:完成方法
def camera_to_field( point):
        # 传入相机坐标系的点云[x,y,z]，返回赛场坐标系的点云[x,y,z]
        # 传入的是一个np.array格式的点云，返回的也是一个np.array格式的点云
        # 直接乘以外参矩阵即可
        point = np.hstack((point, 1))
        point = np.dot(camera_to_field_matrix, point)
        # 把最后的1删掉
        point = point[:3]
        return point

if __name__ == "__main__":
    global width, height, frame
    
    frame = cv2.imread('img.png')
    back = frame.copy()
    width = frame.shape[1]
    height = frame.shape[0]
    # 调用函数读取相机参数
    load_camera_parameters('./parameters/ost.yaml')
    print('camera_matrix:',camera_matrix)
    print('distortion_coefficients:',distortion_coefficients)
    load_lidar_camera_parameters('./parameters/lidar_camera.txt')
    print('lidar2camera:',lidar2camera)
    # 读取pcds.txt
    # 使用numpy加载点云文件
    pc = np.loadtxt('pcds.txt')
    show_pcd_info(pc)
    
    # 将点云转换到相机坐标系下
    pc = pcd_lidar2camera(pc)
    
    # np.savetxt("pcds_camera.txt", pc)
    _anchor = Anchor()

    pp = PointsPicker()
    pp.caller(frame, _anchor)
    while len(_anchor) < 4:
        pp.resume(_anchor)

    cv2.waitKey(1000)
    print('final')
    
    
    
    # 逆时针
    true_points = np.array([
        [400., 300, 8410],
        [400., 0.0, 8410],
        [0.0, 0.0, 8290],
        [0.0, 300., 8290]
    ], dtype=np.float32)
    pixel_points = np.array(_anchor.vertexes, dtype=np.float32)
    print(pixel_points)
    _, rotation_vector, translation_vector = cv2.solvePnP(true_points, pixel_points,
                                                                      camera_matrix, distortion_coefficients,
                                                        flags=cv2.SOLVEPNP_P3P)
    if _:
        print('solvePnP success')
    else:
        print('solvePnP failed')
        exit(0)
    print(rotation_vector,translation_vector)
    rotation_matrix = cv2.Rodrigues(rotation_vector)[0]  # 从赛场到相机的旋转矩阵
    # 构造其次变换矩阵
    transformation_matrix = np.hstack((rotation_matrix, translation_vector.reshape(-1, 1)))  # 创建包含R和T的3x4矩阵

    transformation_matrix = np.vstack((transformation_matrix, [0, 0, 0, 1]))  # 添加一个[0, 0, 0, 1]行向量
    print("transformation_matrix",transformation_matrix)
    
    # 获得从相机坐标系到赛场坐标系的矩阵，通过求逆
    camera_to_field_matrix = np.linalg.inv(transformation_matrix)
    # 将赛场坐标系的平移部分转为m
    camera_to_field_matrix[:3, 3] /= 1000

    print(camera_to_field_matrix)
    
    
    # 创建一个窗体框选出鼠标为中心10x10的区域
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("frame", 1440, 1080)
    cv2.imshow("frame", frame)
    
    cv2.setMouseCallback("frame", mouse_callback)
    
    
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
    img_z = np.full((height, width), np.inf)
    valid = np.bitwise_and(np.bitwise_and((u >= 0), (u < width)),
                               np.bitwise_and((v >= 0), (v < height)))
    
    u, v, z = u[valid], v[valid], z[valid]
    uvz = np.array([u, v, z]).T
    
    print('--------------',uvz.shape)
    while True:
        show = back.copy()
        
        selected_points = get_points_in_box(pc, (cur_mouse[0] - 5, cur_mouse[1] - 5, cur_mouse[0] + 5, cur_mouse[1] + 5))
        if len(selected_points) == 0:
                continue
        box_pcd = o3d.geometry.PointCloud()
        box_pcd.points = o3d.utility.Vector3dVector(selected_points)
        # 点云过滤
        box_pcd = filter(box_pcd)
        # 获取box_pcd的中心点
        cluster_result = cluster(box_pcd) # 也就6帧变7帧，还是启用
        _, center = cluster_result
        # # 如果聚类结果为空，则用中值取点
        if center[0] == 0 and center[1] == 0 and center[2] == 0:
                center = get_center_mid_distance(box_pcd)

        cv2.putText(show, f"Center: {center}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        field_xyz = camera_to_field(center)
        

        cv2.putText(show, f"Field: {field_xyz}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        # cv2.putText(show, f"Depth: {mean_depth}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        # cv2.putText(show, f"Point: {point_xyz}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.circle(show, cur_mouse, 5, (0, 0, 255), -1)
        
        cv2.imshow("frame", show)
        if cv2.waitKey(16) & 0xFF == ord('q'):
            break
    # 用于计算鼠标点击的点的深度
    
    
    
    
    
    
    
    
    
    