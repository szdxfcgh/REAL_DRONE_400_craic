# CRAIC 比赛当天运行手册

本手册用于比赛当天在机载电脑上按顺序完成无桨检查、K230 串口检查、STM32 投放检查、真实传感器 topic 检查、dry-run 和 guarded real launch。所有硬件步骤默认无桨执行，未完成安全确认前不要进入真实飞行。

## 0. 安全确认

- 无桨测试：所有台架测试、`dry_run:=true` 测试和舵机测试都必须先拆桨。
- 遥控器接管：遥控器必须开机、对频正常，飞手随时准备接管。
- 急停：确认现场人员都知道急停方式，起飞前再次口头确认。
- 舵机空载测试后再装物块：先空载发布 1/2/3 路投放命令，确认动作和 ACK 正常，再安装物块。
- 未完成 K230、STM32、真实传感器、规划器、px4ctrl、遥控器接管和急停检查前，不要设置 `dry_run:=false`。

## 1. 机载电脑准备

在机载 Ubuntu/ROS Noetic 电脑上，从工作空间根目录打开新终端执行：

```bash
cd ~/craic_dev_for_onboard
source /opt/ros/noetic/setup.bash
bash scripts/prepare_onboard_workspace.sh
source devel/setup.bash
```

ROS master 基础检查：

```bash
roscore
```

如果其他终端出现 `Unable to communicate with master`，保持 `roscore` 运行，确认每个终端都执行过 `source devel/setup.bash`，并检查 `ROS_MASTER_URI` 是否指向机载电脑。

## 2. K230 测试

连接 K230 USB 串口，确认设备节点：

```bash
ls /dev/ttyACM0
sudo chmod a+rw /dev/ttyACM0
```

启动 K230 串口节点：

```bash
roslaunch craic_mission k230_serial_node.launch serial_port:=/dev/ttyACM0 baud_rate:=115200
```

在其他已 source 的终端检查 K230 必须发布的 topic：

```bash
rostopic echo /craic/qr_result
rostopic echo /craic/target_detected
rostopic echo /craic/landing_marker
rostopic echo /craic/special_target
```

K230 发来的有效消息应以 `ROS_MSG:` 开头。如果节点日志出现 `Unknown K230 ROS_MSG type`，先修正 K230 端消息类型，再继续 dry-run 或真实运行。

## 3. STM32 投放测试

连接 STM32 USB 串口，确认实际串口号：

```bash
ls /dev/ttyUSB*
sudo chmod a+rw /dev/ttyUSB0
```

如果实际设备不是 `/dev/ttyUSB0`，下面命令中的 `serial_port` 要同步改成实际值：

```bash
roslaunch craic_mission drop_controller_serial.launch serial_port:=/dev/ttyUSB0 baud_rate:=115200
```

监听投放状态：

```bash
rostopic echo /craic/drop_status
```

先进行舵机空载链路测试：

```bash
rosrun craic_mission test_drop_sequence.py
```

该脚本会依次发布 `/craic/drop_cmd` 的 `data=1`、`data=2`、`data=3`，并等待 `/craic/drop_status` 返回 `ACK:DROP:1`、`ACK:DROP:2`、`ACK:DROP:3`。正常日志应包含 `sent DROP:1`、`got ACK:DROP:1` 等信息；如果任意 ACK 超过默认 3 秒未收到，脚本会以非 0 退出码结束。

需要调整间隔或超时时可执行：

```bash
rosrun craic_mission test_drop_sequence.py _drop_ids:="[1,2,3]" _interval:=1.0 _timeout:=3.0
```

手动兜底命令：

```bash
rostopic pub /craic/drop_cmd std_msgs/Int32 "data: 1" -1
rostopic pub /craic/drop_cmd std_msgs/Int32 "data: 2" -1
rostopic pub /craic/drop_cmd std_msgs/Int32 "data: 3" -1
```

期望 `/craic/drop_status` 出现 `PONG`、`ACK:DROP:1`、`ACK:DROP:2` 或 `ACK:DROP:3`。如果出现 `ERR:TIMEOUT:DROP:n` 或比赛记录中描述的 `drop ACK timeout`，停止安装物块，先检查 STM32 供电、固件 ACK、波特率和串口线。

## 4. 真实传感器 Topic 检查

真实传感器和飞控相关节点启动后，先检查这些 topic，再启动任务：

```bash
rostopic hz /Odom_high_freq
rostopic hz /cloud_registered
rostopic echo /position_cmd
rostopic info /px4ctrl/takeoff_land
```

必须重点确认：

- `/Odom_high_freq`
- `/cloud_registered`
- `/position_cmd`
- `/px4ctrl/takeoff_land`

如果 `/Odom_high_freq` 没有 publisher 或频率异常，不要继续真实运行。

## 5. Dry-Run 测试

先启动完整 dry-run：

```bash
roslaunch craic_mission competition_dryrun_full.launch
```

在第二个已 source 的终端执行 topic 检查：

```bash
rosrun craic_mission check_craic_topics.py
```

观察任务输出：

```bash
rostopic echo /move_base_simple/goal
rostopic echo /position_cmd
rostopic echo /craic/drop_cmd
rostopic echo /craic_mission_fsm/state
```

手动触发任务的 start topic：

```bash
rostopic pub /craic_mission_fsm/start std_msgs/Empty "{}" -1
```

通过标准：`check_craic_topics.py` 没有异常缺失的关键 publisher，发布 start topic 后 FSM 状态能推进，且能看到规划目标和投放命令按预期出现。

## 6. Real Launch 测试

K230、STM32、真实传感器 topic、遥控器接管和急停都确认后，保持无桨启动 guarded real launch：

```bash
roslaunch craic_mission competition_real.launch dry_run:=true auto_start:=false
```

台架联调阶段保持 `dry_run:=true` 和 `auto_start:=false`。所有操作员确认安全后，再发布 start topic：

```bash
rostopic pub /craic_mission_fsm/start std_msgs/Empty "{}" -1
```

真实飞行前，再重复一次真实传感器 topic 检查，并由飞手确认遥控器可以接管。

## 7. 常见问题

| 问题 | 处理 |
| --- | --- |
| `Unable to communicate with master` | 启动或检查 `roscore`，重新 `source devel/setup.bash`，确认 `ROS_MASTER_URI`。 |
| `Permission denied /dev/ttyACM0` | 执行 `sudo chmod a+rw /dev/ttyACM0`，必要时重新插拔 K230。 |
| `device disconnected` | 检查 USB 线、供电、插头固定，以及设备是否从 `/dev/ttyACM0` 或 `/dev/ttyUSB0` 变成其他编号。 |
| `Unknown K230 ROS_MSG type` | 检查 K230 输出，支持的类型包括 `qr`、`target`、`ring`、`special`、`landing`。 |
| `drop ACK timeout` | 检查 STM32 供电、固件 ACK、串口号、波特率、舵机负载和串口线。 |
| no publisher for `Odom_high_freq` | 先启动真实里程计来源，用 `rostopic hz /Odom_high_freq` 确认后再启动任务。 |

## 8. 比赛当天启动顺序

1. 拆桨，物块先不装到舵机上。
2. 机载电脑执行 `source /opt/ros/noetic/setup.bash`、`bash scripts/prepare_onboard_workspace.sh`、`source devel/setup.bash`。
3. K230 连接 `/dev/ttyACM0`，启动 `k230_serial_node.launch`，确认 `/craic/qr_result`、`/craic/target_detected`、`/craic/landing_marker`、`/craic/special_target`。
4. STM32 执行 `ls /dev/ttyUSB*`，必要时 `chmod`，启动 `drop_controller_serial.launch`，监听 `/craic/drop_status`，运行 `test_drop_sequence.py` 确认 `data: 1/2/3` 都收到 ACK。
5. 检查真实传感器 topic：`/Odom_high_freq`、`/cloud_registered`、`/position_cmd`、`/px4ctrl/takeoff_land`。
6. 启动 `competition_dryrun_full.launch`，运行 `check_craic_topics.py`，发布 start topic。
7. 无桨启动 `competition_real.launch dry_run:=true auto_start:=false`。
8. 空载投放测试通过后再装物块。
9. 遥控器接管、急停和现场口令确认后，才允许进入真实飞行准备。
