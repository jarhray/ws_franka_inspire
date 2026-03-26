# experiment_logger

`experiment_logger` subscribes to key runtime streams and stores them as JSONL files for replay-oriented debugging.

## Recorded streams

- `RobotObservation` -> `observation.jsonl`
- `ArmAction` -> `arm_action.jsonl`
- `HandAction` -> `hand_action.jsonl`
- Arm executor status (`std_msgs/String`) -> `arm_status.jsonl`
- Arm executor error (`std_msgs/String`) -> `arm_error.jsonl`

Each launch creates a session folder under `log_dir`, such as `~/.ros/experiment_logs/20260326_120102`.

## Main parameters

- `enabled` (bool, default `true`)
- `log_dir` (string, default `~/.ros/experiment_logs`)
- `run_name` (string, default `""`)
- `flush_every_n` (int, default `20`)
- topic parameters:
  - `observation_topic`
  - `arm_action_topic`
  - `hand_action_topic`
  - `arm_status_topic`
  - `arm_error_topic`
