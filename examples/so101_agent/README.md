# SO101 Agent 控制接口

为能看图、执行代码的 Agent 提供四个工具：`observe`、`move_joints`、`set_gripper`、`hold`。
默认使用 `mock` 后端，不导入机器人 SDK、不打开串口/摄像头。
另提供显式选择的 `so101` 真机后端，复用既有从臂标定和双摄配置；目前只完成离线验证。
模型服务不在这一层，调用者可通过 Python 或本机 HTTP 使用工具。

这是 AA-Robotics 风格的控制闭环原型，不是 AA 的原始接口或评测复现成绩。
模拟图像为两张关节数值仪表图，关节反馈由限速数值模型生成，不模拟碰撞、重力、物体或抓取。
`simulated: true` 会出现在观测和动作响应中；模拟限位不能当作实机标定。

## 运行

Python 3.12，仅依赖标准库。无需安装完整 Carrot、Ray、LeRobot 或模型 SDK。
从仓库根目录运行一次完整 HTTP 离线演示：

```bash
python -m examples.so101_agent.demo --output-dir outputs/so101_agent_demo
```

输出目录必须尚不存在，包含两路前后图片、`events.jsonl` 请求/响应与轨迹、`summary.json`。
演示依次完成观测、肩部小幅移动、重新观测、夹爪调整、hold、终态观测，随后关闭服务。
重复运行应使用新的输出目录。

启动长期服务：

```bash
python -m examples.so101_agent --backend mock --port 8767 \
  --output-dir outputs/so101_agent_session
```

只监听 `127.0.0.1`；`--port 0` 可自动选择空闲端口，实际 URL 在首行输出。
Ctrl+C 或 SIGTERM 会关闭命令入口、中断动作，再尝试保持模拟关节并退出。
不要把这个实验 HTTP 服务代理到公网；目前没有用户鉴权或持久化命令去重。

## SO101 后端与已有配置

真机适配目前针对 macOS、Python 3.12、LeRobot 0.6.1、feetech-servo-sdk 1.0.0、
OpenCV 4.13、PyObjC 和既有 `uvc-util`。这些依赖不进入 Carrot 训练核心；
mock 和文件检查仍只需标准库。请使用已有机器人环境，不在此入口安装/升级依赖。

输入沿用三个本地文件：teleop JSON 的 `robot`（必须为 `so101_follower`、`use_degrees=true`）、
标定目录内的 `<robot.id>.json`、版本1的 `camera_settings.json`（必须含 `top`、`wrist`）。
夹爪是标定百分比，五轴为标定区间中点相对角度。只使用从臂信息，不连接主臂。
配置和标定保持在原目录，公共仓库不保存本机路径或标定值。

先检查文件；以下命令**不枚举或打开设备，也不运行 UVC 工具**：

```bash
python -m examples.so101_agent --backend so101 --check-config \
  --robot-config /path/to/teleop/config.json \
  --camera-settings /path/to/camera/camera_settings.json \
  --calibration-dir /path/to/calibration/robots/so_follower
```

`--calibration-dir` 可由已有 JSON 中的 `robot.calibration_dir` 提供。
UVC 工具默认取相机配置同目录的 `tools/uvc-util`，也可用 `--uvc-tool` 指定已有可执行文件。
检查输出的 `connected=false` 仅证明文件兼容，不证明设备在线或设备内的标定一致。

设备连接、现场就绪且既有遥控/推理/相机预览已释放设备后，才能运行下面的**真实连接命令**：

```bash
python -m examples.so101_agent --backend so101 \
  --robot-config /path/to/teleop/config.json \
  --camera-settings /path/to/camera/camera_settings.json \
  --calibration-dir /path/to/calibration/robots/so_follower \
  --port 8767 --output-dir outputs/so101_read_only_session
```

默认 `motion_enabled=false`：连接底层电机总线、校验设备标定/位置模式/故障状态和原始位置，
打开双摄并恢复其尺寸、帧率、旋转和曝光；`observe` 可用。
`move_joints`、`set_gripper`、`hold` 都返回 `read_only`（403），退出不写电机、不改变扭矩。
这里的“只读”指电机；相机会设置保存的参数并核对回读值。

完成只读实测后，有现场动作授权时才在**新会话**增加 `--enable-motion`。
该选项会先取得双摄画面，再把当前实测原始位置写成保持目标并回读核对，随后仅使能原本未使能的电机。
这是有电机写入的启动过程，即使还未调用移动工具，也不能当作离线或只读运行。
没有通过 HTTP 远程使能的工具，不调用 `SO101Follower.connect/configure`，不改 PID、模式、标定、EEPROM Lock，
不执行自动复位。退出时先关命令入口并 hold，再收尾请求、关闭相机和串口；保留扭矩。

串口使用同一 tty/cu 别名锁、打开前占用检查、OS 独占打开；不抢占现有客户端。
双摄按保存的 VID:PID 匹配唯一 AVFoundation 设备，重新解析 OpenCV 索引，
不把旧 `index_or_path` 当作插拔后身份凭证。缺失、歧义或两路映射相同都会拒绝启动。
每路采集运行在独立进程，通过非阻塞共享缓存返回最新 PNG；卡住或退出的采集器不能阻塞 hold。
启动有20秒等待预算，缓存帧超过1.5秒拒绝观测。图像时间戳是软件开始读取时的 monotonic 时间，
不是相机硬件曝光时间；USB/驱动缓存仍需上机检查，两摄不承诺同步。

电机位置在归一化前先校验原始标定范围，避免 SDK 对夹爪的裁剪隐藏越界。
目标转换取最近编码器整数刻度（五轴每刻度约0.088°；夹爪为100/标定刻度跨度），
反复 hold 不会产生向下取整漂移。执行器的浮点目标限速/限步仍有最多半个刻度的量化误差。
串口使用0.25秒写入/数据包超时和有限读重试；这不是硬实时制动保证。
会话日志记录配置路径、实际相机索引/曝光、使能状态、工具调用和退出 hold 结果。

## HTTP 协议

- `GET /health`：会话、单位、限位、执行参数、`motion_enabled`、`closed`、当前命令和故障。
- `GET /tools`：四个工具的 JSON Schema（`name`、`description`、`input_schema`）。
- `POST /observe`、`POST /move_joints`、`POST /set_gripper`、`POST /hold`：JSON 参数对象。
- 仅接受 `application/json`；拒绝额外/缺失字段、重复 JSON 键、NaN/Infinity 和超大请求。
- 使用直接的本机 HTTP 客户端；带 `Origin` 或非本机 `Host` 的请求会被拒绝。

例如读取观测：

```bash
curl --fail-with-body http://127.0.0.1:8767/observe \
  -H 'Content-Type: application/json' -d '{}'
```

工具参数如下，HTTP JSON 与 Python 关键字参数相同：

| 工具 | 必需参数 | 结果 |
| --- | --- | --- |
| `observe()` | 无 | 观测编号、实测关节、top/wrist PNG、各自序号及采集时间 |
| `move_joints(...)` | `target`, `duration_s`, `observation_id`, `command_id` | 到位/取消/故障状态、目标、实测终点、误差与每步轨迹 |
| `set_gripper(...)` | `value`, `duration_s`, `observation_id`, `command_id` | 保持五个身体关节的实测位置，仅改变夹爪目标 |
| `hold()` | 无 | 中断的命令编号、保持目标、是否成功写入；不卸力 |

`target` 必须含且仅含以下六个键，不接受数组、`.pos` 后缀或隐式单位转换：

```json
{
  "shoulder_pan": 2.0,
  "shoulder_lift": 0.0,
  "elbow_flex": 0.0,
  "wrist_flex": 0.0,
  "wrist_roll": 0.0,
  "gripper": 50.0
}
```

前五个值为 **degrees**；夹爪为 **0–100 的标定范围比例**，不是度、毫米或夹持力。
以上仅为模拟初始姿态附近的示例，不是实机动作指令。

## Python 用法

```python
from examples.so101_agent import AgentController, MockBackend

agent = AgentController(MockBackend())
observation = agent.observe()
target = {**observation["positions"], "shoulder_pan": 2.0}
result = agent.move_joints(
    target=target,
    duration_s=0.5,
    observation_id=observation["observation_id"],
    command_id="trial-1-move-1",
)
if result["status"] != "reached":
    raise RuntimeError(result)

observation = agent.observe()
result = agent.set_gripper(
    value=45.0,
    duration_s=1.0,
    observation_id=observation["observation_id"],
    command_id="trial-1-gripper-1",
)
held = agent.hold()
```

图像在 `observation["images"]["top"或"wrist"]["data_base64"]`，MIME 为 `image/png`。
时间是同一会话的 monotonic seconds，不是 UTC；图像有各自采集时间，
`camera_skew_s` 显式报告两路时间差，不声称两摄同步。

## 运动与失败语义

动作请求会阻塞至终态。HTTP 使用独立线程，使另一个请求能够调用 `hold` 中断动作。
本地执行器独占后端 I/O，不由模型逐个生成控制周期。后端 I/O 本身须有有限超时；
`hold` 无法中断正在底层驱动里无限阻塞的读写。

- `duration_s` 是名义轨迹时间；太短而超过速度上限会在运动前拒绝，实际运行另有到位等待预算。
- 默认20 Hz，最大目标速度8单位/秒，单次目标变化≤0.5，目标领先反馈≤3。
  同一共享进度控制整条关节轨迹，慢轴限制整体进展。参数见 `MotionConfig` 和 `/health`。
- 目标超出范围直接拒绝，不静默裁剪。需要连续3次实测反馈在容差内才返回 `reached`。
- 新动作要求最新观测编号，默认观测有效期30秒；取观测时图像/关节帧龄须≤1.5秒。
  新观测、一次动作、hold，或观测后明显的关节变化都会使旧编号不可用于新动作。
- 观测只读；观测失败会废弃之前的编号。执行动作期间不持续运行视觉避障。
- 每个动作使用独立 `command_id`。同一会话内相同参数的重试返回原结果；不同参数重用编号返回409。
  运行中的重复请求返回 `command_in_progress`，不会再次运动。缓存满时拒绝新动作，绝不淘汰旧编号。
- 重启后 `session_id` 改变，缓存不跨重启保留；重启后应重新观察并建立新会话，不能盲目重放旧动作。
- `hold` 取消当前轨迹，尝试读取位置并写入保持目标。写入成功不等于机械停止保证；
  读取/写入失败明确返回 `failed`。它不触发复位或卸力，也不清除已锁定的故障。
- 执行中停滞、超时、读写失败时尝试hold并锁定运动。仍可observe/hold，后续动作返回 `faulted`；
  需要排查后重建会话才能恢复，没有隐藏的自动恢复。运行中检测到失能不会自动重新使能。
- `set_gripper` 未到位可能意味着物体阻挡或其他故障；它不会据此返回“抓取成功”。

HTTP 200 仅表示工具已返回：动作结果还必须检查 `status`，可能为
`reached`、`cancelled`、`stalled`、`timeout` 或 `failed`；hold 为 `held` 或 `failed`。
请求被拒绝时返回4xx和 `error.code`，例如 `stale_observation`、`speed_limit`、`busy`。
硬件异常详情保存在动作结果的 `failure` 和 `hold` 字段；JSONL保留调用参数与响应，包含轨迹。

## 验证及下一阶段

```bash
PYTHONPATH="$PWD:$PWD/src" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python -m pytest -q --noconftest -p no:cacheprovider tests/so101_agent
ruff check examples/so101_agent tests/so101_agent
```

测试覆盖真实loopback HTTP、重复命令、hold并发中断、单位/范围、陈旧观测、停滞、超时、
故障锁定及Python优化模式下的外部输入校验。真机适配测试替换电机、相机、UVC和串口边界，
校验现有 SDK 的离线构造/单位换算、配置加载、只读零写入、使能顺序、失能/标定不符、
相机身份/曝光/旋转、跨进程帧缓存、卡住的采集器、关闭时取消及资源清理。
SDK/图像编码兼容性用例需要已有 LeRobot/serial/OpenCV/numpy；未安装时标为 skip。

真机后端已实现，真实连接、画面检查、运动和抓取效果仍为 **NOT RUN**。
下一步需要连接从臂和双摄，先做只读检查，再单独安排受控小幅动作。
离线测试不需要接设备。关节限速/限位不提供碰撞检测或安全抓取保证。
