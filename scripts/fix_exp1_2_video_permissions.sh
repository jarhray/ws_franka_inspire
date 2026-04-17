#!/usr/bin/env bash
# 修复 LeRobot exp1_2 数据集下 videos/ 的读权限，解决非属主用户（如 jhr）解码 mp4 时出现 Permission denied。
#
# 根因常见为：视频由其它账号写入且 umask 过严，得到 -rw------- (600)。
#
# 用法（二选一）：
#   sudo bash /path/to/ws_franka_inspire/scripts/fix_exp1_2_video_permissions.sh
#   # 或数据属主账号（如 test）：
#   bash /path/to/ws_franka_inspire/scripts/fix_exp1_2_video_permissions.sh
#
# 默认目录：/home/test/franka_ros2_ws/data_recorded/exp1_2/videos
# 若你的数据在别处，可覆盖环境变量 DATASET_VIDEOS_ROOT。

set -euo pipefail

DATASET_VIDEOS_ROOT="${DATASET_VIDEOS_ROOT:-/home/test/franka_ros2_ws/data_recorded/exp1_2/videos}"

if [[ ! -d "$DATASET_VIDEOS_ROOT" ]]; then
  echo "ERROR: 目录不存在: $DATASET_VIDEOS_ROOT" >&2
  exit 1
fi

echo "Fixing permissions under: $DATASET_VIDEOS_ROOT"

while IFS= read -r -d '' d; do
  chmod 755 "$d"
done < <(find "$DATASET_VIDEOS_ROOT" -type d -print0)

while IFS= read -r -d '' f; do
  chmod 644 "$f"
done < <(find "$DATASET_VIDEOS_ROOT" -type f -print0)

echo "Done. Example: ls -l \"\$(find '$DATASET_VIDEOS_ROOT' -name '*.mp4' | head -1)\""
echo "Then re-run act_exp1_2_offline_diag.py (non --parquet-only) for one frame."
