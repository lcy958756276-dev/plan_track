#!/usr/bin/env bash
# check_nav.sh - 2D Nav Goal 诊断脚本
# 用法: bash check_nav.sh
# 在 run_debug.sh 启动完毕后，另开终端运行

WORKSPACE_DIR="$(cd "$(dirname "$0")/ros_motion_planning" && pwd)"
source "$WORKSPACE_DIR/devel/setup.bash" 2>/dev/null

LOG="$WORKSPACE_DIR/log/nav_check.log"
echo "========================================" > "$LOG"
echo "  2D Nav Goal 诊断 $(date)" >> "$LOG"
echo "========================================" >> "$LOG"

echo "诊断中... 结果写入 log/nav_check.log"

# 1. 进程检查
echo "" >> "$LOG"
echo "[1] 关键进程:" >> "$LOG"
for p in move_base map_server gzserver; do
    pid=$(pgrep -f "$p" | head -1)
    if [ -n "$pid" ]; then
        echo "  ✅ $p (PID=$pid)" >> "$LOG"
    else
        echo "  ❌ $p (未运行)" >> "$LOG"
    fi
done

# 2. 关键话题
echo "" >> "$LOG"
echo "[2] 关键话题:" >> "$LOG"
for topic in /map /move_base_simple/goal /scan /scan_fixed /odom /tf /move_base/global_costmap/costmap; do
    if rostopic info "$topic" 2>/dev/null | grep -q "Type:"; then
        echo "  ✅ $topic" >> "$LOG"
    else
        echo "  ❌ $topic" >> "$LOG"
    fi
done

# 3. /scan_fixed 更新频率（等3秒看有没有数据来）
echo "" >> "$LOG"
echo "[3] /scan_fixed 是否在更新（监听3秒）:" >> "$LOG"
SCAN_COUNT=$(timeout 3 rostopic echo /scan_fixed -n5 --noarr 2>/dev/null | grep -c "header:" || true)
if [ "$SCAN_COUNT" -gt 0 ]; then
    echo "  ✅ 3秒内收到 $SCAN_COUNT 帧数据" >> "$LOG"
else
    echo "  ❌ 3秒内无数据" >> "$LOG"
fi

# 4. TF 树
echo "" >> "$LOG"
echo "[4] TF 树:" >> "$LOG"
for pair in "map odom" "odom base_footprint" "base_footprint base_link" "base_link base_scan"; do
    src=$(echo $pair | cut -d' ' -f1)
    tgt=$(echo $pair | cut -d' ' -f2)
    result=$(rosrun tf tf_echo "$src" "$tgt" 2>/dev/null | head -8)
    if echo "$result" | grep -q "Translation"; then
        trans=$(echo "$result" | grep "Translation" | head -1)
        echo "  ✅ $src → $tgt  $trans" >> "$LOG"
    else
        echo "  ❌ $src → $tgt (不存在)" >> "$LOG"
    fi
done

# 5. move_base 服务
echo "" >> "$LOG"
echo "[5] move_base 服务:" >> "$LOG"
rosservice list 2>/dev/null | grep move_base >> "$LOG" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "  ❌ 无 move_base 服务" >> "$LOG"
fi

# 6. /map 尺寸
echo "" >> "$LOG"
echo "[6] /map 信息:" >> "$LOG"
MAP_INFO=$(rostopic echo /map -n1 --noarr 2>/dev/null)
if [ -n "$MAP_INFO" ]; then
    W=$(echo "$MAP_INFO" | grep -oP 'width: \K\d+')
    H=$(echo "$MAP_INFO" | grep -oP 'height: \K\d+')
    echo "  width=$W height=$H" >> "$LOG"
else
    echo "  ❌ 无数据" >> "$LOG"
fi

echo "" >> "$LOG"
echo "诊断完成 → $LOG"
echo "内容:"
cat "$LOG"
