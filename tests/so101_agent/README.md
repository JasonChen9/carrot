# SO101 Agent 离线验证

实现与运行方式见 [Agent 接口说明](../../examples/so101_agent/README.md)。
运行环境为 Python 3.12；实现仅使用标准库，测试需要 pytest。

```bash
PYTHONPATH="$PWD:$PWD/src" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python -m pytest -q --noconftest -p no:cacheprovider tests/so101_agent
ruff check examples/so101_agent tests/so101_agent
python -m examples.so101_agent.demo --output-dir /tmp/so101_agent_demo_run
```

演示输出目录必须不存在。测试不安装依赖、不加载GPU模型、不打开串口或摄像头。
HTTP测试实际绑定本机临时端口，设备接口由 `MockBackend` 提供。

## 保护的行为

- 图片可解码、相机命名、时间戳、单位及实测关节与目标值的区别。
- 完整具名目标、非有限值/布尔值/错误单位形式、限位与速度校验，在写入前拒绝无效输入。
- 共享插值进度、每步变化/速度/反馈领先约束、连续到位采样。
- 夹爪操作保持其他五轴；夹爪受阻不被当作成功抓取。
- 过期、被替代、动作后或外部位置变化后的观测不能用于新动作。
- 成功、失败和取消结果的命令去重；不同请求重用ID失败；缓存满时不淘汰旧ID。
- hold在独立线程或HTTP连接中取消阻塞动作，原轨迹随后不再下发。
- 超时、停滞、读写故障、失败hold、KeyboardInterrupt、故障锁定和明确的失败结果。
- HTTP四接口与工具schema、实际持久日志、错误请求、请求体上限、Origin/Host限制。
- `python -O`仍然拒绝非法输入；模拟接口不加载lerobot/cv2/serial/ray/torch。

## 2026-11-18 验证记录

状态：**PASS（离线）**。

- Python 3.12.15、pytest 9.1.1、Ruff 0.17.4；使用本机已有环境，无依赖安装或升级。
- 测试：`59 passed in 0.48s`。
- Ruff check、format检查、`git diff --check`、CLI帮助检查通过。
- 独立CLI演示通过真实loopback HTTP完成observe、move_joints、set_gripper、hold；
  最终模拟肩旋为2°、夹爪为45%，分别返回`reached`、`reached`、`held`。
- `--backend robot`在参数解析阶段被拒绝，未创建输出目录。
- 真机、摄像头、GPU、真实Agent模型调用：**NOT RUN**。

这些结果验证控制接口与模拟执行行为，不证明真实机械臂的标定、运动精度、避碰或任务成功率。
