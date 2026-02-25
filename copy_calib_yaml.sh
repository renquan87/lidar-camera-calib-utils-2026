#!/bin/bash

# 标定文件复制脚本
# 将标定完成的ost.yaml文件复制到parameters目录

# 设置路径
CALIB_DIR="/data/projects/radar/lidar_camera_calib_utils"
PARAMS_DIR="$CALIB_DIR/parameters"
BACKUP_DIR="$PARAMS_DIR/backups"
TEMP_CALIB="/tmp/calibrationdata.tar.gz"
TARGET_FILE="$PARAMS_DIR/ost.yaml"

# 创建备份目录
mkdir -p "$BACKUP_DIR"

echo "=== 标定文件复制脚本 ==="

# 检查是否存在标定压缩包
if [ ! -f "$TEMP_CALIB" ]; then
    echo "错误：未找到标定文件 $TEMP_CALIB"
    echo "请先完成标定并点击SAVE保存文件"
    exit 1
fi

# 备份当前文件
if [ -f "$TARGET_FILE" ]; then
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    BACKUP_FILE="$BACKUP_DIR/ost.yaml.backup_$TIMESTAMP"
    cp "$TARGET_FILE" "$BACKUP_FILE"
    echo "已备份旧文件到: $BACKUP_FILE"
fi

# 创建临时解压目录
TEMP_EXTRACT_DIR=$(mktemp -d)

# 解压并复制文件
tar -xzf "$TEMP_CALIB" -C "$TEMP_EXTRACT_DIR"
OST_FILE=$(find "$TEMP_EXTRACT_DIR" -name "ost.yaml" | head -1)

if [ -n "$OST_FILE" ] && [ -f "$OST_FILE" ]; then
    cp "$OST_FILE" "$TARGET_FILE"
    echo "新标定文件已复制到: $TARGET_FILE"
else
    echo "错误：在压缩包中未找到ost.yaml文件"
    rm -rf "$TEMP_EXTRACT_DIR"
    exit 1
fi

# 清理临时文件
rm -rf "$TEMP_EXTRACT_DIR"

echo "标定文件复制完成！"