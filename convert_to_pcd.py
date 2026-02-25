import numpy as np
import open3d as o3d
import os
import sys

def show_pcd_info(pc):
    x = pc[:, 0]
    y = pc[:, 1]
    z = pc[:, 2]
    print('x: ', x.min(), x.max())
    print('y: ', y.min(), y.max())
    print('z: ', z.min(), z.max())

def convert_txt_to_pcd(input_file="data/pcds.txt", output_file="data/pcds.pcd"):
    """
    将txt格式的点云数据转换为PCD格式
    
    Args:
        input_file: 输入的txt文件路径
        output_file: 输出的PCD文件路径
    """
    # 检查输入文件是否存在
    if not os.path.exists(input_file):
        print(f"Error: Input file {input_file} does not exist!")
        return False
    
    # 确保输出目录存在
    output_dir = os.path.dirname(output_file)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    
    try:
        # 读取txt点云数据
        print(f"Loading point cloud from {input_file}...")
        pc = np.loadtxt(input_file)
        
        # 检查数据格式
        if pc.ndim != 2 or pc.shape[1] != 3:
            print(f"Error: Invalid point cloud format. Expected Nx3 array, got {pc.shape}")
            return False
        
        # 显示点云信息
        print(f"Loaded {len(pc)} points")
        show_pcd_info(pc)
        
        # 创建Open3D点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc)
        
        # 保存为PCD格式
        print(f"Saving PCD file to {output_file}...")
        o3d.io.write_point_cloud(output_file, pcd)
        
        print("Conversion completed successfully!")
        return True
        
    except Exception as e:
        print(f"Error during conversion: {str(e)}")
        return False

if __name__ == "__main__":
    # 设置输入输出文件路径
    input_file = "data/pcds.txt"
    output_file = "data/pcds.pcd"
    
    # 执行转换
    success = convert_txt_to_pcd(input_file, output_file)
    
    if success:
        # 可选：验证转换结果
        try:
            print("\nVerifying converted PCD file...")
            pcd_loaded = o3d.io.read_point_cloud(output_file)
            print(f"Loaded {len(pcd_loaded.points)} points from PCD file")
            
            # 显示加载的点云信息
            pc_loaded = np.asarray(pcd_loaded.points)
            show_pcd_info(pc_loaded)
        except Exception as e:
            print(f"Error verifying PCD file: {str(e)}")
    else:
        print("Conversion failed!")