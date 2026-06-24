#!/usr/bin/env bash
# test.sh — 诊断脚本
# 在小车电脑上执行: bash test.sh
# 会自动生成 test_*.log，然后可以 cat 看或者发给我

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="$SCRIPT_DIR/log"
mkdir -p "$LOG_DIR"
TS=$(date +%Y%m%d_%H%M%S)

echo "===== test.sh 诊断 $TS =====" | tee "$LOG_DIR/test_$TS.log"

# 1. 确认 robot_description 中的 base_scan_joint
echo "" | tee -a "$LOG_DIR/test_$TS.log"
echo "=== [1] robot_description → base_scan_joint ===" | tee -a "$LOG_DIR/test_$TS.log"
rosparam get robot_description 2>/dev/null | grep -A 4 "base_scan_joint" | tee -a "$LOG_DIR/test_$TS.log"
if [ $? -ne 0 ]; then
  echo "❌ rosparam get robot_description 失败" | tee -a "$LOG_DIR/test_$TS.log"
fi

# 2. TF 树 — 生成 pdf 并转文本摘要
echo "" | tee -a "$LOG_DIR/test_$TS.log"
echo "=== [2] TF 树摘要 ===" | tee -a "$LOG_DIR/test_$TS.log"
rosrun tf view_frames 2>/dev/null
if [ -f frames.pdf ]; then
  mv frames.pdf "$LOG_DIR/frames_$TS.pdf"
  echo "✅ TF 树已保存: log/frames_$TS.pdf" | tee -a "$LOG_DIR/test_$TS.log"
else
  echo "⚠️  view_frames 生成失败" | tee -a "$LOG_DIR/test_$TS.log"
fi

# 3. TF: odom → base_scan
echo "" | tee -a "$LOG_DIR/test_$TS.log"
echo "=== [3] tf_echo odom → base_scan ===" | tee -a "$LOG_DIR/test_$TS.log"
timeout 3 rosrun tf tf_echo odom base_scan 2>&1 | tee -a "$LOG_DIR/test_$TS.log" || echo "⚠️  tf_echo 超时/失败" | tee -a "$LOG_DIR/test_$TS.log"

# 4. TF: base_footprint → base_scan
echo "" | tee -a "$LOG_DIR/test_$TS.log"
echo "=== [4] tf_echo base_footprint → base_scan ===" | tee -a "$LOG_DIR/test_$TS.log"
timeout 3 rosrun tf tf_echo base_footprint base_scan 2>&1 | tee -a "$LOG_DIR/test_$TS.log" || echo "⚠️  tf_echo 超时/失败" | tee -a "$LOG_DIR/test_$TS.log"

# 5. 确认节点存活
echo "" | tee -a "$LOG_DIR/test_$TS.log"
echo "=== [5] 关键节点 ===" | tee -a "$LOG_DIR/test_$TS.log"
rosnode list 2>/dev/null | tee -a "$LOG_DIR/test_$TS.log"

# 6. 话题列表
echo "" | tee -a "$LOG_DIR/test_$TS.log"
echo "=== [6] 话题列表 ===" | tee -a "$LOG_DIR/test_$TS.log"
rostopic list 2>/dev/null | tee -a "$LOG_DIR/test_$TS.log"

# 7. /map 是否有数据
echo "" | tee -a "$LOG_DIR/test_$TS.log"
echo "=== [7] /map 话题 ===" | tee -a "$LOG_DIR/test_$TS.log"
rostopic info /map 2>&1 | tee -a "$LOG_DIR/test_$TS.log"

# 8. /scan 是否有数据
echo "" | tee -a "$LOG_DIR/test_$TS.log"
echo "=== [8] /scan 话题 ===" | tee -a "$LOG_DIR/test_$TS.log"
rostopic info /scan 2>&1 | tee -a "$LOG_DIR/test_$TS.log"

# 9. gmapping 日志尾部
echo "" | tee -a "$LOG_DIR/test_$TS.log"
echo "=== [9] gmapping.log 最新20行 ===" | tee -a "$LOG_DIR/test_$TS.log"
tail -20 "$LOG_DIR/gmapping.log" 2>/dev/null | tee -a "$LOG_DIR/test_$TS.log"

# 10. mapper 日志尾部
echo "" | tee -a "$LOG_DIR/test_$TS.log"
echo "=== [10] mapper.log 最新20行 ===" | tee -a "$LOG_DIR/test_$TS.log"
tail -20 "$LOG_DIR/mapper.log" 2>/dev/null | tee -a "$LOG_DIR/test_$TS.log"

echo "" | tee -a "$LOG_DIR/test_$TS.log"
echo "===== 诊断完成 =====" | tee -a "$LOG_DIR/test_$TS.log"
echo "结果文件: $LOG_DIR/test_$TS.log"
echo "TF 树 PDF: $LOG_DIR/frames_$TS.pdf (如果生成成功)"
