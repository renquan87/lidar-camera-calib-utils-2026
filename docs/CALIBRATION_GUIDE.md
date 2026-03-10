# 激光雷达-相机标定完整指南

> 本文档面向 hnurm_radar 雷达站项目，详细解释标定的原理、流程、产出和应用。

---

## 一、为什么需要标定？

雷达站方案二（lidar_scheme）的核心工作流程是：

1. 相机拍到图像 → YOLO 检测出机器人的 2D 像素框
2. 激光雷达扫描出 3D 点云
3. 把点云投影到图像上，找到 2D 框内对应的 3D 点 → 得到机器人的 3D 坐标

第 3 步需要知道两个东西：
- **相机内参**：相机自身的光学参数（焦距、光心、畸变），决定了"3D 世界中的一个点会落在图像的哪个像素上"
- **雷达-相机外参**：激光雷达和相机之间的相对位置和朝向（旋转矩阵 R + 平移向量 T），决定了"雷达坐标系中的一个点，在相机坐标系中是什么位置"

没有这两组参数，点云和图像就对不上，定位就完全不准。

---

## 二、两个标定项目的关系

| 项目 | 路径 | 做什么 | 产出 |
|------|------|--------|------|
| lidar_camera_calib_utils | `/data/projects/radar/lidar_camera_calib_utils` | 相机内参标定 + 验证工具 | 相机内参（ost.yaml）+ 外参验证 |
| direct_visual_lidar_calibration_ws | `/data/projects/radar/direct_visual_lidar_calibration_ws` | 雷达-相机外参标定 | 外参矩阵 T_lidar_camera（四元数形式） |

**标定顺序**：先内参，后外参。外参标定依赖内参结果。

**数据流向**：

```
[相机内参标定]                    [雷达-相机外参标定]
lidar_camera_calib_utils          direct_visual_lidar_calibration_ws
        │                              │
        ▼                              ▼
parameters/ost.yaml              preprocessed/calib.json
(camera_matrix, distortion)      (T_lidar_camera 四元数)
        │                              │
        │                              ▼
        │                    四元数→矩阵 在线转换工具
        │                              │
        │                              ▼
        │                    T_lidar_camera 4×4 矩阵
        │                              │
        │                              ▼
        │                    求逆 → T_camera_lidar 4×4 矩阵
        │                              │
        │                              ▼
        │                    parameters/lidar_camera.txt
        │                              │
        ▼                              ▼
        └──────────┬───────────────────┘
                   ▼
    hnurm_radar/configs/converter_config.yaml
    ├── intrinsic: fx, fy, cx, cy  ← 来自 ost.yaml
    ├── distortion: data           ← 来自 ost.yaml
    └── extrinsic: R, T            ← 来自 lidar_camera.txt
```

---

## 三、什么时候需要重新标定？

### 必须重新标定的情况

| 变化 | 需要重做内参？ | 需要重做外参？ |
|------|:---:|:---:|
| 换了相机 | ✅ | ✅ |
| 换了镜头 | ✅ | ✅ |
| 调了相机焦距/光圈 | ✅ | ✅ |
| 换了激光雷达 | ❌ | ✅ |
| 相机或雷达的安装位置/角度变了 | ❌ | ✅ |
| 从赛场搬到实验室（安装位置变了） | ❌ | ✅ |
| 相机和雷达都没动，只是换了场地 | ❌ | ❌ |

### 不需要重新标定的情况

- 相机和雷达刚性固定在同一个支架上，整体搬动但相对位置没变 → 不需要重新标定
- 只是换了检测模型（YOLO 权重）→ 不需要
- 只是改了配置参数（如 EKF 参数）→ 不需要

### 实验室测试

**如果相机和雷达的相对安装位置与赛场一样**（比如固定在同一个支架上整体搬过来），则不需要重新标定。

**如果重新安装了相机或雷达**（哪怕只是微调了角度），则必须重新做外参标定。

---

## 四、相机内参标定（本项目）

本项目支持两种标定方式：**普通棋盘格**和 **ChArUco 板**（推荐）。

### 4.0 两种标定方式对比

| | 普通棋盘格 (`calib.launch.py`) | ChArUco 板 (`charuco_calib.launch.py`) |
|---|---|---|
| **标定板** | 8×11 普通黑白棋盘格（格子 50mm） | 12×9 ChArUco 板（格子 30mm，标记 22.5mm） |
| **反光容忍度** | 差，反光区域的角点会全部失效 | **好**，每个角点独立检测，部分反光不影响其余角点 |
| **遮挡容忍度** | 必须全部角点可见 | **部分遮挡也能使用** |
| **精度** | 依赖 cornerSubPix 精化，受材质影响大 | ArUco 标记辅助定位，更稳定 |
| **推荐场景** | 哑光标定板 + 均匀光照 | **反光材料 / 光照不均匀 / 一般场景（推荐）** |

### 4.1 原理

相机内参描述了相机的光学特性：

- **焦距 (fx, fy)**：镜头的放大倍率，单位是像素。值越大，视野越窄，远处物体越大
- **光心 (cx, cy)**：图像中心点的像素坐标，理想情况下在图像正中间
- **畸变系数 (k1, k2, p1, p2, k3)**：镜头导致的图像变形（直线变弯曲），用于校正

这些参数通过拍摄已知尺寸的棋盘格标定板，利用多张不同角度的照片求解。

### 4.2 方式一：普通棋盘格标定

适用于哑光材质的棋盘格标定板。

```bash
# 1. 构建项目
cd /data/projects/radar/lidar_camera_calib_utils
rm -rf build/ log/ install/
colcon build --symlink-install
source install/setup.bash

# 2. 启动标定程序
ros2 launch hnurm_camera calib.launch.py

# 3. 在相机视野内移动棋盘格标定板
#    - 覆盖整个视野（上下左右四个角和中间）
#    - 尝试不同角度（正面、倾斜 30°、倾斜 45°）
#    - 尝试不同距离（近、中、远）
#    - 界面上的进度条会逐渐变满

# 4. 当 "CALIBRATE" 按钮变为可点击状态时，点击它
#    等待计算完成（可能需要几十秒）

# 5. 标定完成后查看 RMS 重投影误差（右侧面板显示）
#    - < 0.5 优秀，< 1.0 可用，> 1.0 建议重新标定

# 6. 点击 "COMMIT" 自动保存到 parameters/ost.yaml（旧文件自动备份到 backups/）
#    或点击 "SAVE" 保存到 /tmp/calibrationdata.tar.gz

# 7. Ctrl+C 结束程序
```

### 4.3 方式二：ChArUco 板标定（推荐）

**推荐使用此方式**，特别是当标定板材料有反光时。ChArUco 板结合了棋盘格的亚像素精度和 ArUco 标记的鲁棒检测，即使部分区域被遮挡或反光也能正常标定。

**使用的 ChArUco 板参数**（与 hnuvision_ros2 项目共用同一块板）：
- 格子数：12 × 9
- 格子边长：30mm
- ArUco 标记边长：22.5mm
- ArUco 字典：5x5_1000

```bash
# 1. 构建项目（如果已构建可跳过）
cd /data/projects/radar/lidar_camera_calib_utils
colcon build --symlink-install
source install/setup.bash

# 2. 启动 ChArUco 标定程序
ros2 launch hnurm_camera charuco_calib.launch.py

# 3. 在相机视野内移动 ChArUco 标定板
#    - 同样需要覆盖整个视野、不同角度、不同距离
#    - 与棋盘格不同：即使有部分反光或遮挡也会被采集
#    - 界面上的进度条会逐渐变满

# 4. 当 "CALIBRATE" 按钮变为可点击状态时，点击它
#    等待计算完成

# 5. 标定完成后查看 RMS 重投影误差（右侧面板显示）

# 6. 点击 "COMMIT" 自动保存到 parameters/ost.yaml
#    或点击 "SAVE" 保存到 /tmp/calibrationdata.tar.gz

# 7. Ctrl+C 结束程序
```

> **注意**：如果没有 ChArUco 板，可以使用 OpenCV 生成并打印一张：
> ```bash
> python3 -c "
> import cv2
> aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
> board = cv2.aruco.CharucoBoard((12, 9), 0.03, 0.0225, aruco_dict)
> img = board.generateImage((2400, 1800), marginSize=50)
> cv2.imwrite('charuco_board_12x9.png', img)
> print('已生成 charuco_board_12x9.png，按实际尺寸打印（格子 30mm）')
> "
> ```

### 4.4 产出文件与保存方式

标定完成后有两种保存方式：

#### 方式 A：COMMIT（推荐）

点击 **COMMIT** 按钮，标定结果会通过 `set_camera_info` 服务自动保存到项目目录：

```
parameters/ost.yaml              ← 最新标定结果（自动覆盖）
parameters/backups/ost_20260310_153022.yaml  ← 旧文件自动备份（带时间戳）
```

此方式由 `camera_info_url` 参数控制，launch 文件中已配置好，无需手动操作。

#### 方式 B：SAVE + 手动复制

点击 **SAVE** 按钮，结果保存到 `/tmp/calibrationdata.tar.gz`，然后手动提取：

```bash
./copy_calib_yaml.sh
```

#### ost.yaml 内容示例：

```yaml
camera_matrix:
  data: [3331.28587, 0., 1482.18506,    # fx, 0, cx
         0., 3316.39571, 1193.51941,     # 0, fy, cy
         0., 0., 1.]                     # 0, 0, 1
distortion_coefficients:
  data: [-0.121699, 0.203650, -0.009301, -0.002396, 0.000000]  # k1, k2, p1, p2, k3
```

### 4.5 如何应用到 hnurm_radar

将 `ost.yaml` 中的数值手动填入 `hnurm_radar/configs/converter_config.yaml`：

```yaml
calib:
  intrinsic:
    cx: 1482.18506    # ← ost.yaml camera_matrix 第 1 行第 3 个
    cy: 1193.51941    # ← ost.yaml camera_matrix 第 2 行第 3 个
    fx: 3331.28587    # ← ost.yaml camera_matrix 第 1 行第 1 个
    fy: 3316.39571    # ← ost.yaml camera_matrix 第 2 行第 2 个
  distortion:
    data: [-0.121699, 0.203650, -0.009301, -0.002396, 0.000000]  # ← 直接复制
```

---

## 五、验证工具

本项目还提供了几个验证工具：

### pnp_demo.py
用 PnP（Perspective-n-Point）方法验证标定结果。给定一些已知的 3D-2D 对应点，反算出外参，看是否与标定结果一致。

### crossValidation.py
交叉验证：用标定得到的外参将点云投影到图像上，检查投影是否对齐。如果点云的颜色和图像的物体边缘对得上，说明标定准确。

### inspector.py
可视化检查工具，直观查看标定效果。

```bash
cd /data/projects/radar/lidar_camera_calib_utils
source install/setup.bash

python3 pnp_demo.py          # PnP 验证
python3 crossValidation.py   # 交叉验证（点云投影到图像）
python3 inspector.py          # 可视化检查
```

---

## 六、参数文件一览

| 文件 | 内容 | 来源 |
|------|------|------|
| `parameters/ost.yaml` | 相机内参（焦距、光心、畸变） | 相机内参标定产出 |
| `parameters/lidar_camera.txt` | T_camera_lidar 4×4 变换矩阵 | calib_ws 外参标定产出（经转换） |
| `parameters/backups/ost_*.yaml` | 历史标定结果备份（带时间戳） | COMMIT 时自动备份 / copy_calib_yaml.sh |

---

## 七、常见问题

**Q: 标定板用什么规格？**
A: 本项目支持两种标定板：
- **普通棋盘格**：8×11（内角点 7×10），格子边长 50mm，`calib.launch.py`
- **ChArUco 板（推荐）**：12×9，格子 30mm，标记 22.5mm，字典 5x5_1000，`charuco_calib.launch.py`

**Q: ChArUco 板和普通棋盘格有什么区别？**
A: ChArUco 板在每个白色方格内嵌入了 ArUco 二维码标记。这使得即使标定板部分被遮挡或有反光，仍然可以正确检测到可见的角点。普通棋盘格在有任何角点不可见时整帧数据将丢弃。

**Q: 标定时 CALIBRATE 按钮一直灰色？**
A: 需要采集足够多不同角度和位置的样本。确保标定板在视野的各个区域都出现过，并且有不同的倾斜角度。

**Q: 标定误差多少算合格？**
A: 标定完成后界面右侧会显示 RMS 重投影误差（reprojection error），一般 < 0.5 像素算优秀，< 1.0 像素算可用，> 1.0 建议重新标定。

**Q: COMMIT 和 SAVE 有什么区别？**
A: **COMMIT** 会通过 ROS 的 `set_camera_info` 服务将标定结果直接保存到项目目录 `parameters/ost.yaml`（旧文件自动备份到 `parameters/backups/`）。**SAVE** 只是打包保存到 `/tmp/calibrationdata.tar.gz`，需要手动复制。推荐使用 COMMIT。

**Q: 换了分辨率需要重新标定吗？**
A: 如果相机硬件分辨率改变了（比如从 3072×2048 改成 4024×3036），需要重新标定。内参值会随分辨率变化。
