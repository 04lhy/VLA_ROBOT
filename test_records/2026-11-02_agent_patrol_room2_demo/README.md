# 2026-11-02 单房间巡检整合演示（agent × Nav2 × YOLO × OpenVLA）

目标：一条链跑通——**Nav2 到巡检停靠位 → 前置相机 YOLO 认牌子 → agent 决策**
红色→去本房间桌前→dock 精对齐→OpenVLA v4 整段回合（抓杯→倾倒补药→放回→松开）
→**绿色→下一停靠位**；决策全部由 agent_demo 顶层做，子系统只暴露工具接口。

## 整合改动（本轮）

| 文件 | 改动 |
|---|---|
| `vla_gazebo_bridge.py` | ① `/vla_instruction` 任意非空消息即起新回合（同文案也可重触发，全量重置）；② `auto_warm`：回合起手自动发 GT f1 预热位 `[0,0.493,0.084,0.001,-0.5,-0.001]`+张爪+静置（第十轮手工协议内置化），预热期间 `armed` 闸门不收帧；③ `/vla_episode_status` 状态机 IDLE→WARMING→RUNNING→COMPLETE（1Hz 周期发布，供上层阻塞等待）；④ launch 直接带 instruction 的旧 demo 流程兼容（起手也先预热） |
| `base_robot.launch.py` | 自启桥接参数补全=10q 验证配方：`vla_instruction` launch 参数（默认 ''=topic 门控）、`stack_frames:=True`、GT 脱困 ep_0004（share 目录回退 5 级定位数据集，机器无关）、`min_ee_z 0.19` 等 |
| `demo_scene.world` | **room2（东南）牌子由绿翻红**——VLA 工位只验证过东南桌，演示要走红→抓取分支；西北保持绿可演示绿分支。恢复方法见 world 内注释（改回 `0 1 0 1` + colcon build） |
| `agent_demo/tools.py` | ① `vla_manipulate` 真实模式=发布指令后**阻塞轮询** `/vla_episode_status` 到 COMPLETE（420s 上限；防陈旧 COMPLETE：先见 WARMING/RUNNING 才认）；② `dock="room2"` 参数=抓取前 gz teleport 精对齐底盘+杯到 10q 验证位（模型只在固定机位采过数据，Nav2 停靠精度不够）；③ `room2_table` 停靠坐标对齐 VLA 工位 |
| `agent_demo/executor.py` | 红色子计划改为 `去桌前 → 一次 VLA 整段回合`（旧的"抓取杯子/放回杯子"两条中文指令是 OOD，v4 一条演示 episode 本身就走完抓→倒→放） |
| `agent_demo/planner.py` | mock 规划器支持点名房间（"巡检 room2"只巡 room2+回家；不点名=四房全巡） |
| `agent_demo/run_room2_demo.sh` | 一体化演示脚本：清场→v4 服务器→仿真→静置 75s→底盘归位+摆杯（4 杯叠原点需全部挪开，防碾杯 SIGABRT）→导航栈→agent |

## 预期时序

```
agent main.py "巡检 room2" --mock-planner --real-vla-publish --real-navigation --real-detection
 ├─ navigate_to room2   (Nav2, 0,0 → 20.13,-14.89)
 ├─ detect_color room2  (前置相机抓帧 → color_yolo ONNX → red)
 ├─ [Branch] red →
 │   ├─ navigate_to room2_table (14.152,-11.868)
 │   └─ vla_manipulate "pick up the cup and pour" + dock=room2
 │        ├─ gz teleport 精对齐 base+cup（demo 对接辅助）
 │        ├─ /vla_instruction → 桥接 WARMING(自动预热~13s) → RUNNING
 │        └─ 轮询 /vla_episode_status → COMPLETE（整段 57 步 ±GT 脱困）
 └─ navigate_to home
```

## 联调实录（分链路打通，10:20~10:45）

三段子系统在整合环境里逐一实测通过，抓出并修掉 3 个"CLI/库差异"坑：

1. **Nav2 走不通**：`tools._navigate_to_pose` 原用 `ros2 action send_goal` CLI——
   目标到达 bt_navigator 但 goal response 发不回（FastDDS CLI 老毛病，nav.log 实录
   "Failed to send goal response … client will not receive response"），300s 超时、
   机器人一步不动。→ 改为 rclpy ActionClient 子进程（V1.5.0 nav_waypoints.py
   实测路径）。首测又撞 `send_goal_async(PoseStamped)` TypeError——必须
   `NavigateToPose.Goal()` 包装；再撞 Humble 结果类没有 `Result.SUCCESS` 常量——
   用字面 status==4。**修后实测：(0,0)→room2 停靠位成功**（停于 19.96,-14.68，
   22cm/13° 误差——正好由 dock snap 兜底）。
2. **YOLO 红牌确认**：机器人停靠位真实画面 `detect_color('room2') → red conf=0.945`
   （world 翻红 + 前置相机 + ONNX 全链路在整合环境出真值）。
3. **VLA 回合"假失败"**：桥接端实录 WARMING(15s 自动预热)→RUNNING→**COMPLETE
   （67s，62 步，ESCAPE-GT/Traceback=0，杯站立放回 z 名义+0.25mm、位移 3.3cm）**，
   但 agent 420s 等不到状态判失败——`ros2 topic echo <topic> data --once` 把裸
   `data` 当**消息类型**解析失败静默空转。→ 改 `--field data`。auto_warm/状态机/
   重触发语义一次通过。

## 已知风险与对策

- Nav2 目标点被 costmap 判碰撞（桌/杯/墙膨胀）→ 微调 `room2_table` 到走廊点，dock snap 兜底精对齐
- AMCL 停靠误差 ±5~25cm → 不影响抓取（snap 对齐后才起回合），但回合内底盘反冲 ±7mm 级
- 回合预热期间 agent 阻塞在状态轮询（echo --once 包 timeout 8，防桥接死挂）
- gz 11.10.2 挤压 SIGABRT 前科 → 摆位时 3 只闲杯全部挪离原点

## 运行结果（2026-11-02 实测记录）

### 1) 三段链路联调（10:20~10:45，见上一节）全部 ✓

### 2) 接通真实 LLM（11:00~11:30）

- 环境修复：系统 python3 装 `openai`；发行版自带 Brotli 1.0.9 与 httpx2 响应解码
  接口不兼容（`process() takes no keyword arguments`）→ pip 升 Brotli 1.2.1。
- key 存放：`agent_demo/.env`（git 忽略、600 权限），`llm_client` 导入时自动加载，
  环境变量优先；写 `~/.bashrc` 的方案被安全策略拦截，`.env` 更规范。
- smoke：planner 直连返回合法 JSON 计划；`main.py`（真 LLM + 全 mock 工具）rc=0。
- 模型：`deepseek-v4-pro` 思考型单次规划 1~3 分钟 → 账号下最轻量 `deepseek-flash`
  实测一次完整规划 ~15s，`.env` 设 `AGENT_LLM_MODEL=deepseek-flash`。

### 3) 全流程整合跑（真 LLM，11:37~11:52，Step0~Step5 全自动）

- 基础设施全绿：服务器 22s 就绪；桥接门控行确认（深度门 z≤0.241 / GT 脱困）；
  摆位回读精确（底盘 (0,0)、工位杯 14.152,-11.514 名义值）。
- **第 1 段 VLA 回合成功**：11:46:34 WARMING → 11:46:50 RUNNING →
  11:47:59 COMPLETE（executor 红色分支自动插入，auto_warm/状态机在真 LLM+全冷启
  动下一遍过）。
- **暴露问题**：flash 运行时计划比预览多写了一步 `vla_manipulate`（没听提示词的
  "抓取子计划由 executor 自动插入"），11:50:51 同一工位**重复起了第 2 段回合**
  ——即"决策真在 LLM"的代价。应要求停场。

### 4) 策略修订：VLA 工位许可制（当日落地 + 单测）

- 推理服务器整轮巡检**常驻**（7B 重载 1~3 分钟/房，逐站开关浪费时间；空闲零算力）。
- `navigate_to` 发放许可 / `vla_manipulate` 消耗：同一停留点重复操作步无操作跳过
  （`duplicate_skipped`）；回合超时/失败许可回退可重试；**计划收尾 main.py 统一
  关停服务器**（`[Cleanup]`）。
- planner 提示词加硬规则（巡检计划禁输出 vla_manipulate/`*_table`，executor 自动插）。
- 单测序列 `['C','C','T','C']`：去重 1 次、跨房续许可、超时重试全过。

### 5) 待办

- 许可制下全流程重跑一次（红色抓取后应只有一段回合，随后 room3→room4→home 收尾）。
- `DOCK_POSES` 仅 room2 标定；其他房判红可用需各工位补采数据。

## 日志清单（已归档本目录）

- `run.log`：演示脚本阶段日志（含 agent 实时流水——`agent.log` 因进程被停在管道
  flush 前，归档为 0 字节属正常）
- `deploy_server.log`：v4 推理服务器
- `sim.log`：Gazebo+控制器+桥接（两段回合的 WARMING/RUNNING/COMPLETE 全记录）
- `nav.log`：Nav2 bringup + AMCL + RViz
