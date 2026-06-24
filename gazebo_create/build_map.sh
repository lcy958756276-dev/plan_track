#!/usr/bin/env bash
# build_map.sh
# 【在 Jetson 上运行】启动 Gazebo + 小车 + gmapping 建图
# 用键盘控制小车走一圈，扫完保存地图

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
GAZEBO_DIR="$SCRIPT_DIR"

# 替换成你实际的 workspace 路径
WORKSPACE_DIR="$HOME/plan_track/ros_motion_planning"
source "$WORKSPACE_DIR/devel/setup.bash"

# 清理
killall -9 gzserver gzclient 2>/dev/null
sleep 1

# ── 1. 加载车模型 ──
echo "[1] 加载 robot_description..."
rosparam set robot_description "$(cat "$GAZEBO_DIR/urdf/my_car/my_car.urdf")"

# robot_state_publisher（发布 URDF 中的固定 TF）
rosrun robot_state_publisher robot_state_publisher &
PID_RSP=$!
sleep 1

# ── 2. 启动 Gazebo ──
echo "[2] 启动 Gazebo + final.world..."
gzserver "$GAZEBO_DIR/worlds/final.world" __name:=gz_debug &
PID_GZ=$!

for i in $(seq 1 15); do
    if rosservice list 2>/dev/null | grep -q "/gazebo/set_model_state"; then
        echo "  ✅ Gazebo 就绪"
        break
    fi
    sleep 1
done

# Gazebo GUI（可选，开来看见小车位置）
gzclient &
PID_GUI=$!
sleep 2

# ── 3. 在 Gazebo 中生成小车 ──
echo "[3] 生成小车模型..."
rosrun gazebo_ros spawn_model -urdf \
    -param robot_description \
    -model my_car \
    -x 0.0 -y 0.0 -z 0.0
sleep 2

# ── 4. 启动建图桥接（cmd_vel → set_model_state + /odom）──
echo "[4] 启动 gazebo_mapper.py..."
python3 "$GAZEBO_DIR/scripts/gazebo_mapper.py" &
PID_MAPPER=$!
sleep 2

# ── 5. 启动 gmapping ──
echo "[5] 启动 gmapping..."
rosrun gmapping slam_gmapping scan:=/scan _odom_frame:=odom _map_update_interval:=1.0 &
PID_GMAPPING=$!
sleep 2

# ── 6. RViz 查看建图进度 ──
echo "[6] 启动 RViz..."
rosrun rviz rviz -d "$GAZEBO_DIR/rviz/sim_env.rviz" &
PID_RVIZ=$!

echo ""
echo "========================================"
echo "  ✅ 建图环境已启动"
echo "  控制: 另开终端运行以下命令"
echo "    python3 $GAZEBO_DIR/scripts/key_teleop.py"
echo ""
echo "  按键:"
echo "    i=前进  k=停止  j=左转  l=右转"
echo "    ,=后退  u=左前  o=右前"
echo ""
echo "  在地图中走一圈后，另开终端保存地图:"
echo "    rosrun map_server map_saver -f $GAZEBO_DIR/maps/my_map"
echo ""
echo "  停止: kill $PID_GZ $PID_GUI $PID_RSP $PID_MAPPER $PID_GMAPPING $PID_RVIZ"
echo "========================================"
