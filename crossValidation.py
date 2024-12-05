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

def generate_depth_image(pc): # 传入的pcd,返回的是一个深度图

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
    return img_z


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
    return uvz

def generate_3d_points(pc):
    uvz = generate_depth_map(pc)
    
    # 提取frame在u,v点的rgb
    bgr = frame[uvz[:, 1].astype(int), uvz[:, 0].astype(int)]
    # 将bgr翻转
    bgr = bgr[:, [2, 1, 0]]

    # 将RGB从0-255范围转换到0-1范围
    bgr = bgr / 255.0

    # 根据深度图，解算原点云
    xyz = np.zeros((uvz.shape[0], 3))
    xyz[:, 0] = uvz[:, 2] * (uvz[:, 0] - camera_matrix[0, 2]) / camera_matrix[0, 0]
    xyz[:, 1] = uvz[:, 2] * (uvz[:, 1] - camera_matrix[1, 2]) / camera_matrix[1, 1]
    xyz[:, 2] = uvz[:, 2]
    # 将xyzrgb合并
    xyzrgb = np.hstack((xyz, bgr))
    # 保存点云
    # np.savetxt("3d_points.txt", xyzrgb)
    # 生成点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(bgr)
    o3d.io.write_point_cloud("colored_point_cloud.ply", pcd)
    o3d.visualization.draw_geometries([pcd])
    


if __name__ == "__main__":
    global width, height, frame
    
    frame = cv2.imread('img.png')
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
    
    
    
    
    # 生成深度图
    depth_map = generate_depth_image(pc)
    # 保存深度图
    cv2.imwrite('depth_map.png', depth_map)
    
    
    # 生成3d点云
    generate_3d_points(pc)
    
    
    
    
    