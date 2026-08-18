# hex_device_test
hex_device 多设备耐久性测试工具
**如果你在使用hex_device，而不是hex_driver_robot，请拉取v1分支**

> ⚠️ 仅限内部使用。

---

## 🚀 快速开始

### 1. 配置虚拟环境
``` bash
mkdir ~/hex_test && cd ~/hex_test
uv venv --python 3.11
source ~/hex_test/.venv/bin/activate
```

### 2. 安装
``` bash
git clone https://github.com/TaiGong26/hex_device_test.git
cd hex_device_test
uv pip install -e .
```

### 3. 使用
```bash
python test/arm_test_v2.py --url ws://xx.xx.xx.xx:8439
```

#### 参数说明
- --url                     必选项，输入一个连接地址
- --robot-type              机械臂类型["Archer_y6", "Firefly_y6"], 默认Archer_y6, 仅支持同时开启一种机械形
- --traj-json               导入的轨迹点
- --interp                  选择插值方式['s_curve', 'linear'], 默认liner, 仅支持在导入的waypoint中使用。
- --view                    开始可视化
- --temp-csv-dir            输出路径；默认None，不输出


#### 使用须知
- 如果遇到错误情况，会使得所有设备刹车，此时控制台有提示 "[dev x]: running -> brake  reason : xxxx "
- 后续仅支持退出操作：进行一次ctrl c
- 然后等待程序回收退出
- 程序退出过程中不要多次按 ctrl c，因为此时机械臂正在返回home位置。
- 程序退出后，状态会保存到 `--temp-csv-dir` 下的 CSV 中，可以通过 `column -s, -t <your_path>.csv | less` 进行查看。(xxx为时间)

---

## 🎙️ 轨迹录制（record）

`record/arm_record.py` 负责录制机械臂关节轨迹，输出 JSON 文件，供 `test/arm_test_v2.py` 的 `--traj-json` 回放（耐久性测试）。本工具只负责"录"，回放由 arm_test_v2 完成。

> 注意：需在 `record/` 目录内运行（使用了相对导入）。

### 两种录制模式

| 模式 | 说明 |
|------|------|
| `record`（默认） | 手动逐点录制。启动后机械臂保持零位，通过键盘录点：`r` 记录当前关节位置（立即写盘），`c` 清空已记录的点。适合"摆好姿势 → 按 r 录一个点"的手动轨迹采集。按键监听走 stdin，必须在真实终端运行（不能后台/重定向）。 |
| `stream` | 定时流式录制。按 `--samp-rate` 定时采样机械臂状态，自动连续录点。适合直接跟随手柄/示教器的连续运动，无需手动按键。 |

### 命令行参数

| 参数 | 取值 | 默认 | 说明 |
|------|------|------|------|
| `--mode` | `record` \| `stream` | `record` | 录制模式 |
| `--ctrl-rate` | int | `1000` | 控制循环频率 Hz（传给 HexRate） |
| `--samp-rate` | int | `100` | 采样频率 Hz（stream 模式生效） |
| `--ip` | str | 必填 | 机械臂 IP |
| `--port` | int | 必填 | 机械臂端口 |
| `--output` | str | 必填 | 输出轨迹 JSON 路径 |
| `--robot-type` | `archer_y6` \| `firefly_y6` | `archer_y6` | 机械臂型号 |
| `--grip-type` | `empty` \| `gp80` \| `gr100` \| `gp100` | `empty` | 夹爪类型 |

### 用法示例

```bash
# 手动逐点录制
python arm_record.py --mode record --ip 192.168.1.100 --port 8439 \
    --output /tmp/traj.json

# 流式录制（100Hz 采样）
python arm_record.py --mode stream --ip 192.168.1.100 --port 8439 \
    --samp-rate 100 --output /tmp/traj_stream.json
```

> 需在 `record/` 目录内执行。Ctrl+C 正常结束：record 模式会闭合 JSON 并修正 info 头部元数据。

### 回放录制的轨迹

```bash
# linear 插值（默认）
python test/arm_test_v2.py --url ws://<IP>:8439 --traj-json /tmp/traj.json

# S 曲线插值
python test/arm_test_v2.py --url ws://<IP>:8439 --traj-json /tmp/traj.json --interp s_curve
```

### 轨迹使用前测试

如果你希望在使用录制轨迹之前测试轨迹效果，可以使用假臂，并开启`archer_view.py`来查看轨迹效果

前置条件：
- `pip install robomeshcat`
- 修改`archer_view.py`的ip
- 修改`archer_view.py`的`URDF_PATH`为urdf
  - urdf可以通过`github.com/hexfellow`获取
- 修改` MESH_DIR `为`URDF_PATH`的父目录

---

## 📄 waypoint JSON 规范

`--traj-json` 导入的轨迹文件为 JSON 格式（由 record 生成，`src/hex_device_test/tools/PointLoader.py` 的 `TaskConfigLoader` 读取）。顶层含 `info`（元数据）与 `point`（逐点数据）：

```json
{
  "info": {
    "start_time_ns": 0,
    "end_time_ns": 1000000000,
    "total_points": 3,
    "dof": 6,
    "robot_type": "archer_y6",
    "gripper_type": "empty"
  },
  "point": {
    "1": { "ts_ns": 0,          "arm": [-0.75, -0.25, 3.0, -1.5, 0, 0], "grip": [] },
    "2": { "ts_ns": 500000000,  "arm": [-0.75, 1.2, 0, 1.5, 0, 2.5],   "grip": [] },
    "3": { "ts_ns": 1000000000, "arm": [-0.75, -0.25, 3.0, -1.5, 0, 0], "grip": [] }
  }
}
```

### info 字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `start_time_ns` | int | 起始时间戳（ns） |
| `end_time_ns` | int | 结束时间戳（ns，相对累计值） |
| `total_points` | int | 总点数 |
| `dof` | int | 自由度，固定为 6 |
| `robot_type` | str | 机械臂型号，如 `"archer_y6"` |
| `gripper_type` | str | 夹爪类型，如 `"empty"` |
| `samp_hz` | int | 采样频率 Hz（仅 stream 模式写入） |

### point 字段

- 键为从 1 开始的连续字符串序号 `"1"` ~ `"N"`，共 `total_points` 个。
- 每点字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `ts_ns` | int | 相对时间戳（ns），首点 = 0 |
| `arm` | float[6] | 6 个关节位置，单位弧度 |
| `grip` | float[] | 夹爪关节位置；无夹爪时为空数组 `[]` |

> 注意：`ts_ns` 是相对首点的时间差，回放时 `TaskConfigLoader.get_timestamps()` 会换算为秒。夹爪轨迹已写入 JSON，但当前回放侧暂不驱动（见 `arm_test_v2.py` 内 `###TODO`）。

---

## 📊 CSV 日志格式

测试时传入 `--temp-csv-dir <dir>` 会把日志写入该目录，程序退出回收后生成两类 CSV 文件。

可用 `column -s, -t <your_path>.csv | less` 对齐查看。

### 1. 汇总报告 `arm_test_{时间}.csv`

每台设备一行，程序退出时由 Coordinator 统一写出。文件名 `arm_test_{YYYY-MM-DD_HH-MM-SS}.csv`，列序如下：

| 列 | 类型 | 说明 |
|----|------|------|
| `start_time` | str | 任务启动时间 `YYYY-MM-DD HH:MM:SS.ffffff` |
| `run_time` | str | 运行时长 `HH:MM:SS` |
| `device_id` | int | 设备编号 |
| `loop_counter` | int | 轨迹循环次数 |
| `motor_{i}_temperature` | float | 第 i 个电机最高温度（℃） |
| `motor_{i}_temperature_min` | float | 第 i 个电机最低温度（℃） |
| `driver_{i}_temperature` | float | 第 i 个驱动器最高温度（℃） |
| `driver_{i}_temperature_min` | float | 第 i 个驱动器最低温度（℃） |
| `errors` | str | 错误列表，用 `" | "` 分隔，格式 `错误@时间` |


### 2. 每设备温度 `temp_dev{id}_{时间}.csv`

每台设备独立写入，约 1Hz 追加一行。**无表头**，第 1 列为时间字符串 `YYYY-MM-DD HH:MM:SS`，后接 N 个电机温度（℃，Archer/Firefly y6 为 6 个）。

> 注意：`log/` 目录下的历史 `arm_test_*.csv` 是旧版本生成的，表头为 `run_time,device_id,state,...`、无 `start_time` 与 `_min` 列，且 `state` 为字符串（如 `Normal`），与当前代码输出不一致。

---

## ⚙️ 功能特性

* 多设备协同调度
* 基于进程的执行模式
* 实时状态追踪
* 支持 CSV 日志记录
* 可扩展的控制器架构

---



## 📌 注意事项

* 专为内部耐久性测试设计
* 依赖 hex_device 运行环境
* 运行测试前请确保设备连接正常
---
