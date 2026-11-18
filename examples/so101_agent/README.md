# SO101 Agent 控制接口（离线版本）

为能看图、执行代码的 Agent 提供四个工具：`observe`、`move_joints`、`set_gripper`、`hold`。
本版本只实现 `mock` 后端；不会导入机器人 SDK、打开串口/摄像头或连接 GPU。
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
不支持 `--backend robot`，也没有串口参数。Ctrl+C 会尝试保持模拟关节并退出。
不要把这个实验 HTTP 服务代理到公网；目前没有用户鉴权或持久化命令去重。

## HTTP 协议

- `GET /health`：会话、单位、限位、执行参数、当前命令和故障。
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
  当前离线版本通过重建模拟会话恢复，没有隐藏的自动恢复。
- `set_gripper` 未到位可能意味着物体阻挡或其他故障；它不会据此返回“抓取成功”。

HTTP 200 仅表示工具已返回：动作结果还必须检查 `status`，可能为
`reached`、`cancelled`、`stalled`、`timeout` 或 `failed`；hold 为 `held` 或 `failed`。
请求被拒绝时返回4xx和 `error.code`，例如 `stale_observation`、`speed_limit`、`busy`。
硬件异常详情保存在动作结果的 `failure` 和 `hold` 字段；JSONL保留调用参数与响应，包含轨迹。

## 验证及下一阶段

```bash
python -m pytest -q tests/so101_agent
ruff check examples/so101_agent tests/so101_agent
```

测试覆盖真实loopback HTTP、重复命令、hold并发中断、单位/范围、陈旧观测、停滞、超时、
故障锁定及Python优化模式下的外部输入校验。所有设备均为模拟后端。

真机适配尚未实现和验证。后续需要接入既有标定及相机配置、有限超时的单串口所有权、
明确的连接/使能流程，再做只读检查和受控小幅动作。届时再连接设备；
本轮离线测试不需要接机械臂或摄像头。关节限速/限位不提供碰撞检测或安全抓取保证。
