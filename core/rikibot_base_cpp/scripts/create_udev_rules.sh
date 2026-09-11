#!/bin/bash
# 安装 udev 规则，把底盘 CH340 串口固定为 /dev/rikibase
# 只需执行一次，之后无论插拔顺序如何，底盘口都是 /dev/rikibase
#
# 注意：必须在【宿主机】上执行。Docker 容器内没有 udev 守护进程，
#       装了也不会生效（udevadm control --reload 会失败）。
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RULES_SRC="${SCRIPT_DIR}/../config/99-rikibase.rules"
RULES_DST="/etc/udev/rules.d/99-rikibase.rules"

echo "复制规则文件: ${RULES_SRC} -> ${RULES_DST}"
sudo cp "${RULES_SRC}" "${RULES_DST}"

echo "重新加载 udev 规则"
sudo udevadm control --reload
sudo udevadm trigger

echo "完成。验证命令: ls -l /dev/rikibase"
