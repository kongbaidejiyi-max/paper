#!/usr/bin/env bash
# 一键关闭所有 tmux ROS 会话

SESSION="ros_all"

# 检查 tmux 是否安装
if ! command -v tmux &> /dev/null; then
    echo "错误: tmux 未安装"
    exit 1
fi

# 检查会话是否存在
if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "会话 '$SESSION' 不存在或已关闭"
    exit 0
fi

echo "正在关闭 ROS 会话: $SESSION"
tmux kill-session -t "$SESSION"

echo "已关闭所有终端窗口"
