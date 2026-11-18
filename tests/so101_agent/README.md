# SO101 Agent 离线验证

实现与运行方式见 [Agent 接口说明](../../examples/so101_agent/README.md)。
运行环境为 Python 3.12；mock/配置检查仅使用标准库，测试需要 pytest。
SDK兼容性及PNG旋转测试还使用已有 LeRobot、Feetech SDK、serial、OpenCV 和 numpy；
缺少这些可选依赖时对应测试标为 skip，本次验证全部执行、无跳过。

```bash
PYTHONPATH="$PWD:$PWD/src" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python -m pytest -q --noconftest -p no:cacheprovider tests/so101_agent
ruff check examples/so101_agent tests/so101_agent
python -m examples.so101_agent.demo --output-dir /tmp/so101_agent_demo_run
```

演示输出目录必须不存在。测试不安装依赖、不加载GPU模型、不打开串口或摄像头。
HTTP测试实际绑定本机临时端口；设备接口由 `MockBackend` 或注入假的原始电机/相机对象提供。
共享缓存测试真实启动一个只发送合成消息的子进程，SDK兼容测试禁止真实串口打开，
UVC子进程调用、设备解析和VideoCapture都替换为fake。

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
- 复用配置加载、重复字段拒绝、显式单位和标定ID/范围、所有标定整数刻度的往返换算。
- 默认只读连接/observe/退出零电机写入；保持、移动和夹爪工具在只读模式返回403。
- 启动前校验模式、标定、原始反馈、故障和双摄；写入原始hold目标并回读后才使能。
- 已使能电机不切换扭矩，启动失败清理资源；运行中失能拒绝写入，不自动重新使能。
- 真机适配层下四工具离线闭环，编码器量化误差与重复hold无漂移；只允许Goal/Torque写入。
- 串口有限超时、独占设置和别名互斥；SDK构造不打开串口，连接口被fake替代。
- 双摄USB身份重排、缺失/歧义、曝光回读失败、四种旋转和参数/画面尺寸不符。
- 实际spawn跨进程缓存传递、卡住的写者不阻塞读取、帧过期、启动超时和采集进程退出清理。
- 服务只读/动作模式启动日志、退出先关闭命令入口、取消后不再访问已关闭总线。
- HTTP端口绑定失败不连接任何设备；`--check-config`在Python优化模式下拒绝加载硬件SDK。

## 2026-11-18 验证记录

状态：**PASS（离线）**。

- Python 3.12.15、pytest 9.1.1、Ruff 0.17.4；使用本机已有环境，无依赖安装或升级。
- 适配兼容性：LeRobot 0.6.1、feetech-servo-sdk 1.0.0、OpenCV 4.13.0。
- 测试：`120 passed in 2.31s`，无skip（包含此前59项接口回归）。
- Ruff check、format检查、`git diff --check`、CLI帮助检查通过。
- 独立CLI演示通过真实loopback HTTP完成observe、move_joints、set_gripper、hold；
  最终模拟肩旋为2°、夹爪为45%，分别返回`reached`、`reached`、`held`。
- 本机现有teleop/双摄/标定文件通过`--check-config`；六轴全部有效原始刻度可往返，
  配置、标定及UVC工具文件SHA256前后相同；该检查不枚举或打开硬件。
- 真机、摄像头、GPU、真实Agent模型调用：**NOT RUN**。

这些结果验证控制接口与模拟执行行为，不证明真实机械臂的标定、运动精度、避碰或任务成功率。
