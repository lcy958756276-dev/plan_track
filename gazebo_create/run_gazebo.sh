#!/usr/bin/env bash
# run_gazebo.sh
# 【参考脚本】把这个文件复制到 Jetson 上执行，不要在本地跑
# 用途：启动 Gazebo GUI 搭建世界
# 使用方式：
#   1. 先 cd 到 gazebo_create 目录
#   2. bash run_gazebo.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 清理
killall -9 gzserver gzclient 2>/dev/null
sleep 1

echo "启动 Gazebo GUI..."
echo ""
echo "  【操作步骤】"
echo "  1. 左边 Insert 标签 → 拖 box/cylinder 到场景"
echo "  2. 选中模型 → 拖箭头移动 / 按 R 旋转 / 按 S 缩放"
echo "  3. 右边 Property 面板精确输入尺寸和位置"
echo "  4. 搭好后 File → Save World As → worlds/my_world.world"
echo ""

gzserver "$SCRIPT_DIR/worlds/empty.world" &
gzclient &

echo "  停止: killall gzserver gzclient"