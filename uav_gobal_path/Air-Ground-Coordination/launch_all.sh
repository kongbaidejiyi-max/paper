#!/usr/bin/env bash
# 一键启动 - 使用 tmux 分屏（单窗口7分屏：3+3+1）

set -euo pipefail

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 会话名称
SESSION="ros_all"

# 检查 tmux 是否安装
if ! command -v tmux &> /dev/null; then
  echo "错误: tmux 未安装"
  echo "请运行: sudo apt-get install tmux"
  exit 1
fi

# 检查目录是否存在
if [[ ! -d "$SCRIPT_DIR/Car" ]]; then
  echo "错误: 找不到目录: $SCRIPT_DIR/Car"
  exit 1
fi
if [[ ! -d "$SCRIPT_DIR/Drone" ]]; then
  echo "错误: 找不到目录: $SCRIPT_DIR/Drone"
  exit 1
fi

# 检查是否已存在会话
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "会话 $SESSION 已存在，正在关闭..."
  tmux kill-session -t "$SESSION"
  sleep 0.3
fi

echo "正在启动所有终端窗口..."

# ==================== 编译工作空间 ====================
build_workspace() {
  local workspace="$1"
  local name="$2"
  echo "编译 $name 工作空间: $workspace"
  (
    source /opt/ros/noetic/setup.bash
    cd "$workspace"
    catkin_make -DCMAKE_BUILD_TYPE=Release
  )
}

echo ""
echo "=========================================="
echo "   编译工作空间"
echo "=========================================="

build_workspace "$SCRIPT_DIR/Car" "Car" &
PID_CAR=$!
build_workspace "$SCRIPT_DIR/Drone" "Drone" &
PID_DRONE=$!

# 逐个 wait，确保任何一个失败都会被 set -e 捕获
wait "$PID_CAR"
wait "$PID_DRONE"

echo "编译完成！"
echo ""

# ==================== 创建分屏布局 ====================
# 目标布局（单窗口7个分屏）:
# ┌─────────┬─────────┬─────────┐
# │   1     │    2    │    3    │
# │  Car    │ Drone   │ Drone   │
# │(ego_pln)│(fitplan)│(fit_pln)│
# ├─────────┼─────────┼─────────┤
# │   4     │    5    │    6    │
# │Car_Vis  │Drone_Vis│  Py_acc │
# ├─────────┴─────────┴─────────┤
# │           7                  │
# │      Py_transform            │
# └──────────────────────────────┘

# 创建会话根 pane
ROOT_PANE="$(tmux new-session -d -s "$SESSION" -P -F "#{pane_id}")"

# 先切出底部横跨全宽的 pane（7）
BOTTOM_PANE="$(tmux split-window -v -t "$ROOT_PANE" -p 25 -P -F "#{pane_id}")"
TOP_PANE="$ROOT_PANE"

# TOP 再切成两行：第一行(1~3) 和 第二行(4~6)
ROW2_PANE="$(tmux split-window -v -t "$TOP_PANE" -p 50 -P -F "#{pane_id}")"
ROW1_PANE="$TOP_PANE"

# ROW1 水平切成三列：1,2,3
PANE_2="$(tmux split-window -h -t "$ROW1_PANE" -p 67 -P -F "#{pane_id}")"
PANE_3="$(tmux split-window -h -t "$PANE_2"     -p 50 -P -F "#{pane_id}")"
PANE_1="$ROW1_PANE"

# ROW2 水平切成三列：4,5,6
PANE_5="$(tmux split-window -h -t "$ROW2_PANE" -p 67 -P -F "#{pane_id}")"
PANE_6="$(tmux split-window -h -t "$PANE_5"    -p 50 -P -F "#{pane_id}")"
PANE_4="$ROW2_PANE"

# 语义映射
CAR_PL_PANE="$PANE_1"
DRONE_FIT_PANE="$PANE_2"
DRONE_PLN_PANE="$PANE_3"
CAR_VIS_PANE="$PANE_4"
DRONE_VIS_PANE="$PANE_5"
PY_ACC_PANE="$PANE_6"
PY_TRANS_PANE="$BOTTOM_PANE"

# 给 pane 设置标题
tmux select-pane -t "$CAR_PL_PANE"    -T "1 Car_planner"
tmux select-pane -t "$DRONE_FIT_PANE" -T "2 Drone_fit"
tmux select-pane -t "$DRONE_PLN_PANE" -T "3 Drone_planner"
tmux select-pane -t "$CAR_VIS_PANE"   -T "4 Car_Vis"
tmux select-pane -t "$DRONE_VIS_PANE" -T "5 Drone_Vis"
tmux select-pane -t "$PY_ACC_PANE"    -T "6 Py_acc"
tmux select-pane -t "$PY_TRANS_PANE"  -T "7 Py_trans"

# 公共环境（按你的原始写法保留）
COMMON_SOURCE="source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && source ~/Project/simu_ws/devel/setup.bash && source devel/setup.bash"

send_run() {
  local pane="$1"
  local dir="$2"
  local label="$3"
  local cmd="$4"

  # 尽量清理一下当前 pane 的状态，避免残留输入/进程影响
  tmux send-keys -t "$pane" C-c

  tmux send-keys -t "$pane" "cd '$dir'" C-m
  tmux send-keys -t "$pane" "$COMMON_SOURCE" C-m
  tmux send-keys -t "$pane" "clear; echo '$label'; $cmd" C-m
}

# [1] Car - ego_planner
send_run "$CAR_PL_PANE" "$SCRIPT_DIR/Car" "[1/7] Car - ego_planner" \
  "roslaunch ego_planner grp_run.launch"

# [2] Drone - fitplane
send_run "$DRONE_FIT_PANE" "$SCRIPT_DIR/Drone" "[2/7] Drone - fitplane" \
  "roslaunch fitplane sim.launch"

# [3] Drone - fitplane_planner
send_run "$DRONE_PLN_PANE" "$SCRIPT_DIR/Drone" "[3/7] Drone - fitplane_planner" \
  "roslaunch fitplane_planner sim.launch"

# [4] Car - visualization
send_run "$CAR_VIS_PANE" "$SCRIPT_DIR/Car" "[4/7] Car - visualization" \
  "roslaunch car_visualization car_visualization.launch"

# [5] Drone - visualization
send_run "$DRONE_VIS_PANE" "$SCRIPT_DIR/Drone" "[5/7] Drone - visualization" \
  "roslaunch drone_visualization drone_visualization.launch"

# [6] Python - acc_point
send_run "$PY_ACC_PANE" "$SCRIPT_DIR/Drone" "[6/7] Python - acc_point" \
  "python3 -u src/fitplane/scripts/acc_point.py"

# [7] Python - transform
send_run "$PY_TRANS_PANE" "$SCRIPT_DIR/Car" "[7/7] Python - transform" \
  "python3 -u src/ego-planner/src/planner/plan_manage/scripts/transform.py"

# 初始选中 pane 1
tmux select-pane -t "$CAR_PL_PANE"

# 显示帮助信息（在外部终端打印）
echo ""
echo "=========================================="
echo "   TMUX 分屏布局 (7个分屏)"
echo "=========================================="
echo ""
echo " 布局说明:"
echo "   ┌─────────┬─────────┬─────────┐"
echo "   │ [1] Car  │ [2]Drone│ [3]Drone│"
echo "   │ego_plann │ fitplane│ fit_pln │"
echo "   ├─────────┼─────────┼─────────┤"
echo "   │ [4]Car_V │ [5]Dr_V │ [6]Py_ac│"
echo "   │ Vis      │ Vis     │ acc_pt  │"
echo "   ├─────────┴─────────┴─────────┤"
echo "   │         [7] Python          │"
echo "   │       (transform)           │"
echo "   └─────────────────────────────┘"
echo ""
echo " 重新连接会话: tmux attach -t $SESSION"
echo " 关闭所有:     tmux kill-session -t $SESSION"
echo ""
echo "=========================================="
echo ""

# 附加到会话
tmux attach-session -t "$SESSION"
