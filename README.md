# LiDAR-Camera Calibration Utils

激光雷达与相机标定工具包，用于标定激光雷达和相机之间的外参矩阵。

## 注意事项

parameters/lidar_camera中是雷达到相机，所以calib.json需要转成 4×4 矩阵，求逆，填入parameters/lidar_camera。

## 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件。


## 引用

如果您在研究中使用了这个工具包，请引用本仓库。



./copy_calib_yaml.sh将相机标定结果复制到param中
