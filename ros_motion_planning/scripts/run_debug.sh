#!/usr/bin/env bash
# run_debug.sh
# 启动所有节点（含 Gazebo，但 Gazebo 不发布 odom TF，由 encoder_odom.py 接管）
# 输出写入 log/ 目录
#
# 与 main.sh 的关系:
#   - main.sh: 原版 roslaunch（Gazebo 正常发布 odom TF，用于纯仿真）
#   - run_debug.sh: 设置 /tmp/.use_encoder_odom 标志位，Gazebo 跳过 odom TF
#   - stop_debug.sh: 清理此标志位

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="$WORKSPACE_DIR/log"

source "$WORKSPACE_DIR/devel/setup.bash"
mkdir -p "$LOG_DIR"

# 强制清理残留的 Gazebo、map_server 进程
killall -9 gzserver gzclient 2>/dev/null
killall map_server 2>/dev/null
sleep 1

# 确保 ROS master 在运行
if ! rostopic list > /dev/null 2>&1; then
    echo "  roscore 未运行，正在启动..."
    roscore \
        >> "$LOG_DIR/run.log" 2>&1 &
    sleep 3
fi

# 清理 ROS master 上的僵尸节点（避免新旧 gzserver 名字冲突）
echo y | rosnode cleanup 2>/dev/null || true
sleep 1

# 清理旧日志
rm -f "$LOG_DIR"/*.log

# 记录启动时间
echo "=== 启动时间: $(date) ===" > "$LOG_DIR/run.log"

# ── 1. 生成动态启动文件 ──
echo "[1/8] 生成动态启动文件..."
echo "[$(date +%H:%M:%S)] [1] main_generate.py start" >> "$LOG_DIR/run.log"
python "$WORKSPACE_DIR/src/plugins/dynamic_xml_config/main_generate.py" user_config.yaml \
    >> "$LOG_DIR/run.log" 2>&1
echo "[$(date +%H:%M:%S)] [1] main_generate.py done" >> "$LOG_DIR/run.log"

# ── 标志位：告诉 Gazebo 不发布 odom TF（物理车模式）──
# main.sh 没有此文件 → Gazebo 正常行为
echo "[2/8] 设置 use_encoder_odom 标志位..."
echo "[$(date +%H:%M:%S)] [2] use_encoder_odom flag set" >> "$LOG_DIR/run.log"
touch /tmp/.use_encoder_odom
echo "  /tmp/.use_encoder_odom → Gazebo diff_drive 将跳过 odom TF"

# ── 3. 加载机器人模型 ──
echo "[3/8] 加载 robot_description + 启动核心节点 ..."
echo "[$(date +%H:%M:%S)] [3] loading robot_description" >> "$LOG_DIR/run.log"

# 加载 SolidWorks 导出的自定义车模型 URDF
ROBOT_URDF="$WORKSPACE_DIR/my_robot/urdf/my_robot.urdf"
rosparam set robot_description "$(cat "$ROBOT_URDF")"
echo "[$(date +%H:%M:%S)] [3] robot_description loaded" >> "$LOG_DIR/run.log"
echo "  robot_description 已加载（自定义车模型）"

# robot_state_publisher（发布 URDF 中固定关节的 TF：base_footprint → base_link → base_scan）
rosrun robot_state_publisher robot_state_publisher __name:=rsp_main \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_RSP=$!
echo "  robot_state_publisher PID=$PID_RSP"

# joint_state_publisher（发布轮子关节状态，让机器人模型显示轮子）
rosrun joint_state_publisher joint_state_publisher \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_JSP=$!
echo "  joint_state_publisher PID=$PID_JSP"

sleep 1

# map_server（提供背景地图 — my_large_map）
MAP_FILE="$WORKSPACE_DIR/../gazebo_create/maps/my_large_map.yaml"
rosrun map_server map_server "$MAP_FILE" \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_MAP=$!
echo "  map_server PID=$PID_MAP"

sleep 1

# 静态 map → odom TF（初始位置偏移在 encoder_odom 中处理）
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_TF=$!
echo "  static_transform_publisher (map→odom) PID=$PID_TF"

# 静态 base_footprint → base_link TF（新模型 URDF 不含 base_footprint，需要补一个）
# rpy=(1.5708 0 -1.5708) 是 SolidWorks 坐标系到 ROS 坐标系的转换
rosrun tf2_ros static_transform_publisher 0 0 0 1.5708 0 -1.5708 base_footprint base_link \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_FP=$!
echo "  static_transform_publisher (base_footprint→base_link) PID=$PID_FP"

sleep 1

# ── 4. 启动 Gazebo（仓库环境）──
echo "[4/8] 启动 Gazebo ..."
echo "[$(date +%H:%M:%S)] [4] starting gzserver" >> "$LOG_DIR/run.log"
WORLD_FILE="$WORKSPACE_DIR/../gazebo_create/worlds/final_two.world"

# 禁用所有模型数据库下载（防 libcurl SSL 超时阻塞 ROS 插件初始化）
# GAZEBO_MODEL_DATABASE_URI 管 Gazebo Fuel，IGN_FUEL_URI 管 Ignition Fuel
export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:1/"
export IGN_FUEL_URI="http://127.0.0.1:1/"

# 用自定义节点名避免与残留 /gazebo 节点冲突（服务路径仍为 /gazebo/*）
GZ_NAME="__name:=gz_debug"
rosrun gazebo_ros gzserver "$WORLD_FILE" "$GZ_NAME" \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_GZSERVER=$!
echo "  gazebo server PID=$PID_GZSERVER (节点名: gz_debug)"

# 等待 Gazebo 的 ROS 服务就绪（超时 15 秒）
echo "  等待 Gazebo ROS 服务就绪..."
for i in $(seq 1 15); do
    if rosservice list 2>/dev/null | grep -q "/gz_debug/set_model_state"; then
        echo "  ✅ Gazebo ROS 服务已就绪（第 ${i} 秒）"
        break
    fi
    sleep 1
done
if ! rosservice list 2>/dev/null | grep -q "/gz_debug/set_model_state"; then
    echo "  ⚠ 警告：15 秒后 /gz_debug/set_model_state 仍未就绪"
    echo "  检查 run.log 中是否有 libcurl 超时阻塞"
fi

# Gazebo GUI
rosrun gazebo_ros gzclient \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_GZCLIENT=$!
echo "  gazebo client PID=$PID_GZCLIENT"

sleep 2

# 在 Gazebo 中生成机器人模型（使用已加载的 robot_description）
echo "[$(date +%H:%M:%S)] [4] spawn_model start" >> "$LOG_DIR/run.log"
echo "  正在生成机器人模型..."
rosrun gazebo_ros spawn_model -urdf \
    -param robot_description \
    -model my_robot \
    -gazebo_namespace /gz_debug \
    -x 0.0 -y -0.8 -z 0.0 \
    >> "$LOG_DIR/run.log" 2>&1
SPAWN_EXIT=$?
echo "[$(date +%H:%M:%S)] [4] spawn_model exit=$SPAWN_EXIT" >> "$LOG_DIR/run.log"
if [ $SPAWN_EXIT -eq 0 ]; then
    echo "  ✅ 机器人已在 Gazebo 中生成"
else
    echo "  ⚠ spawn_model 失败 (exit=$SPAWN_EXIT)"
fi

sleep 2

# ├── RViz ──
RVIZ_FILE="$WORKSPACE_DIR/src/sim_env/rviz/sim_env.rviz"
rosrun rviz rviz -d "$RVIZ_FILE" \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_RVIZ=$!
echo "  rviz PID=$PID_RVIZ"

sleep 2

# ── 恢复 ROS 时间为真实时间 ──
# Gazebo 的 gzserver 内部会设 /use_sim_time=true，导致
# rospy.Time.now() 返回仿真时间（卡在 0），encoder_odom 无法计算 dt。
# 这里显式改回 false，让 read_uart / encoder_odom 用真实时钟。
rosparam set /use_sim_time false
echo "  /use_sim_time → false（编码器/里程计使用真实时钟）"

# ── 5. 启动串口桥接（合并读 tick + 写速度到同一串口） ──
echo "[5/8] 启动 serial_bridge.py (串口桥接：读编码器 + 写 cmd_vel)..."
rosrun encoder_tools serial_bridge.py \
    > "$LOG_DIR/serial_bridge.log" 2>&1 &
PID_READ=$!
echo "  PID=$PID_READ → log/serial_bridge.log"

sleep 2

# ── 6. 启动里程计（初始位置偏移到 my_map_two 空闲区）──
echo "[6/8] 启动 encoder_odom.py (里程计)..."
rosrun encoder_tools encoder_odom.py \
    _initial_x:=0.0 _initial_y:=-0.8 \
    > "$LOG_DIR/encoder_odom.log" 2>&1 &
PID_ODOM=$!
echo "  PID=$PID_ODOM → log/encoder_odom.log"

sleep 1

# ── 7. 启动 Gazebo 同步桥接 ──
echo "[7/8] 启动 gazebo_sync.py (里程计→Gazebo 同步 + LaserScan 时间戳修复)..."
rosrun encoder_tools gazebo_sync.py \
    > "$LOG_DIR/gazebo_sync.log" 2>&1 &
PID_SYNC=$!
echo "  PID=$PID_SYNC → log/gazebo_sync.log"

sleep 2

# ── 7.5 启动 pre_rotate ──
echo "[7.5/8] 启动 pre_rotate.py (新 goal 大角度时原地旋转)..."
rosrun encoder_tools pre_rotate.py \
    > "$LOG_DIR/pre_rotate.log" 2>&1 &
PID_PREROT=$!
echo "  PID=$PID_PREROT"

sleep 1

# ── 8. 启动 move_base（全局规划器）──
echo "[8/8] 启动 move_base（A* 全局规划器，仅规划不跟踪）..."
echo "[$(date +%H:%M:%S)] [8] generating move_base launch file" >> "$LOG_DIR/run.log"

SIM_ENV_DIR="$WORKSPACE_DIR/src/sim_env"
MB_LAUNCH="/tmp/move_base_debug.launch"
cat > "$MB_LAUNCH" << MBEOF
<launch>
  <node pkg="move_base" type="move_base" name="move_base" output="screen">

    <!-- 全局规划器 -->
    <param name="base_global_planner" value="path_planner/PathPlanner"/>
    <param name="PathPlanner/planner_name" value="hybrid_astar"/>

    <!-- 局部规划器（MPC = 模型预测控制，跟踪路径） -->
    <param name="base_local_planner" value="mpc_controller/MPCController"/>

    <!-- 禁用恢复行为（实物车速度低，不会大幅偏离路径） -->
    <param name="recovery_behavior_enabled" value="false"/>

    <!-- move_base 通用参数 -->
    <rosparam command="load" file="$SIM_ENV_DIR/config/move_base_params.yaml"/>

    <!-- 全局 costmap（用 robot-specific 文件，不含 global_costmap: 外层键） -->
    <rosparam command="load" file="$SIM_ENV_DIR/config/robots/my_car/global_costmap_params_my_car.yaml" ns="global_costmap"/>
    <!-- 全局 costmap 插件（含 global_costmap: 外层键 → 加载后自动在 /move_base/global_costmap/ 下） -->
    <rosparam command="load" file="$SIM_ENV_DIR/config/costmap/global_costmap_plugins.yaml"/>
    <param name="global_costmap/obstacle_layer/scan/topic" value="/scan_fixed"/>

    <!-- 局部 costmap（用 robot-specific 文件） -->
    <rosparam command="load" file="$SIM_ENV_DIR/config/robots/my_car/local_costmap_params_my_car.yaml" ns="local_costmap"/>
    <rosparam ns="local_costmap" command="load">
      plugins:
        - {name: obstacle_layer, type: 'costmap_2d::ObstacleLayer'}
        - {name: inflation_layer, type: 'costmap_2d::InflationLayer'}
    </rosparam>

    <!-- map 重映射 -->
    <remap from="map" to="/map"/>
    <!-- goal 重映射（pre_rotate 转发后的话题） -->
    <remap from="move_base_simple/goal" to="/goal_rotated"/>
  </node>
</launch>
MBEOF

echo "[$(date +%H:%M:%S)] [8] starting move_base via roslaunch" >> "$LOG_DIR/run.log"
echo "  启动 move_base 前检查参数状态:" >> "$LOG_DIR/run.log"
rosparam get /move_base/base_global_planner 2>&1 >> "$LOG_DIR/run.log" || echo "  [诊断] base_global_planner 参数未设置" >> "$LOG_DIR/run.log"
rosparam get /move_base/base_local_planner 2>&1 >> "$LOG_DIR/run.log" || echo "  [诊断] base_local_planner 参数未设置" >> "$LOG_DIR/run.log"
rosparam get /move_base/global_costmap/plugins 2>&1 >> "$LOG_DIR/run.log" || echo "  [诊断] global_costmap/plugins 参数未设置" >> "$LOG_DIR/run.log"
echo "  [诊断] rospack plugins nav_core:" >> "$LOG_DIR/run.log"
rospack plugins --attrib=plugin nav_core 2>&1 >> "$LOG_DIR/run.log"

roslaunch "$MB_LAUNCH" \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_MB=$!
echo "  move_base PID=$PID_MB (roslaunch)"
echo "  全局规划器: A* (path_planner/PathPlanner)"
echo "  局部规划器: MPC (模型预测控制，跟踪全局路径)"
echo "  RViz 中点击 2D Nav Goal → 全局路径将显示在地图上"

sleep 3
echo "[$(date +%H:%M:%S)] [8] move_base 启动后状态:" >> "$LOG_DIR/run.log"
echo "  [诊断] 检查 move_base 是否存活:" >> "$LOG_DIR/run.log"
kill -0 "$PID_MB" 2>&1 && echo "  [诊断] move_base 进程存活" >> "$LOG_DIR/run.log" || echo "  [诊断] move_base 进程已死亡" >> "$LOG_DIR/run.log"
echo "  [诊断] rostopic list | grep move_base:" >> "$LOG_DIR/run.log"
rostopic list 2>/dev/null | grep -i move 2>&1 >> "$LOG_DIR/run.log" || echo "  [诊断] 无 move_base 话题" >> "$LOG_DIR/run.log"
echo "  [诊断] rosservice list | grep move_base:" >> "$LOG_DIR/run.log"
rosservice list 2>/dev/null | grep -i move 2>&1 >> "$LOG_DIR/run.log" || echo "  [诊断] 无 move_base 服务" >> "$LOG_DIR/run.log"


sleep 1

# 保存 PID
echo "$PID_MAP"      > "$LOG_DIR/.pid_map"
echo "$PID_RSP"      > "$LOG_DIR/.pid_rsp"
echo "$PID_JSP"      > "$LOG_DIR/.pid_jsp"
echo "$PID_TF"       > "$LOG_DIR/.pid_tf"
echo "$PID_FP"       > "$LOG_DIR/.pid_footprint"
echo "$PID_GZSERVER" > "$LOG_DIR/.pid_gzserver"
echo "$PID_GZCLIENT" > "$LOG_DIR/.pid_gzclient"
echo "$PID_RVIZ"     > "$LOG_DIR/.pid_rviz"
echo "$PID_READ"     > "$LOG_DIR/.pid_read"
echo "$PID_ODOM"     > "$LOG_DIR/.pid_odom"
echo "$PID_SYNC"     > "$LOG_DIR/.pid_sync"
echo "$PID_PREROT"   > "$LOG_DIR/.pid_prerot"
echo "$PID_MB"        > "$LOG_DIR/.pid_move_base"

echo ""
echo "========================================"
echo "  全部已启动，日志文件在 log/ 目录下"
echo ""
echo "  ✅ Gazebo + RViz + 实物编码器同步运行"
echo "  ✅ 2D Nav Goal → A* 全局规划 → MPC 路径跟踪"
echo "  ✅ serial_bridge 合并了串口读写（读tick + 写cmd_vel）"
echo "========================================"
echo ""
echo "  查看里程计日志:  tail -f $LOG_DIR/encoder_odom.log"
echo "  查看编码器日志:  tail -f $LOG_DIR/read_uart.log"
echo ""
echo "  在 RViz 中:"
echo "    1. 点击上方工具栏的 \"2D Nav Goal\""
echo "    2. 在地图上点击目标位置（保持按住，拖动设定朝向）"
echo "    3. 松开 → A* 规划路径并显示在 RViz 中"
echo ""
echo "  发送指令请另开终端运行:"
echo "    cd $WORKSPACE_DIR && source devel/setup.bash"
echo "    rosrun encoder_tools send_uart.py"
echo ""
echo "  停止所有进程:   bash $SCRIPT_DIR/stop_debug.sh"
echo "========================================"
