# Agent Demo

该目录包含一个用于复杂机器人任务的最小 `plan-execute` agent 框架。

当前范围：

- 支持文本、音频文件、定时麦克风录音和流式麦克风输入。
- 默认通过 DeepSeek 文本 planner 规划（或 `--mock-planner` 离线规划）；语音还支持原生 Omni 文件输入和 Omni Realtime 流式输入。
- 使用受限 JSON plan schema。
- Executor 只执行白名单工具。
- `navigate_to`（Nav2 rclpy ActionClient）、`detect_color`（前置相机 + YOLO ONNX）已接真实实现；验证工具仍为 mock 占位。
- `vla_manipulate` 真实模式 = 发布 `/vla_instruction` 起**整段 VLA 回合**并阻塞等待 `/vla_episode_status=COMPLETE`（V2.0.0 起）。
- 四子系统整合巡检 demo 见下方「四子系统整合 demo」一节（一键脚本 `run_room2_demo.sh`）。

## 使用 DeepSeek 兼容 API 运行

```bash
cd agent_demo
export DEEPSEEK_API_KEY="your key"
python3 main.py "去厨房拿一杯水回来" --print-plan
```

也可以把 key 放在 `agent_demo/.env`（已 gitignore，不进仓库），`llm_client`
导入时自动加载，环境变量优先于 `.env`：

```bash
echo 'DEEPSEEK_API_KEY=sk-xxxx' > agent_demo/.env   # 一次性写入，之后直接跑
python3 main.py "去厨房拿一杯水回来" --print-plan
```

默认 API 模板参考 `ds_demo.py`：

- `OpenAI(...)`
- `base_url="https://api.deepseek.com"`（可用 `AGENT_LLM_BASE_URL` / `--base-url` 覆盖）
- 代码默认模型 `deepseek-v4-pro`（思考型，单次规划 1~3 分钟）。**demo 推荐在 `.env`
  里加 `AGENT_LLM_MODEL=deepseek-flash`**（轻量快，实测一次规划 ~15s）
- 依赖：系统 python3 `pip install openai`（若报 `process() takes no keyword
  arguments`，是发行版老 brotli 与 httpx2 不兼容，`pip install -U Brotli` 即解）

## 语音输入

以下命令均在 `agent_demo` 目录执行，相对音频路径以当前工作目录为准。

### 环境与凭据

```bash
cd ~/VLA_ROBOT/agent_demo
python3 -m pip install -r requirements.txt
# 使用麦克风时额外安装；缺少 portaudio.h 时先安装系统包 portaudio19-dev
python3 -m pip install -r requirements-mic.txt
```

首次使用，将 `.env.example` 复制为 `.env` 并填写；已有配置直接加载。程序不会自动读取 `.env`。

| 环境变量 | 用途 |
|---|---|
| `DASHSCOPE_API_KEY` | Fun-ASR / Omni 共用的百炼 API Key |
| `DASHSCOPE_WORKSPACE` | 对应业务空间的 Workspace ID |
| `DASHSCOPE_REGION` | 默认 `cn-beijing`，新加坡使用 `ap-southeast-1` |
| `DEEPSEEK_API_KEY` | ASR 后端调用真实 DeepSeek planner 时需要 |

```bash
set -a
source .env
set +a
```

### 调用方式

| 后端 | 处理链路 | 默认模型 |
|---|---|---|
| `asr`（默认） | 音频 → Fun-ASR → 文本 → 文本 planner | `fun-asr-realtime` / `deepseek-v4-pro` |
| `omni` | 音频文件 → Omni → 转写 + 计划 | `qwen3.8-omni-flash` |
| `omni-realtime` | 流式音频 → Omni Realtime → 转写 + 计划 | `qwen3.8-omni-flash-realtime` |

```bash
# Fun-ASR 文件输入，转写后交给文本 planner
python3 main.py --audio-file ../asr_test/Chinese.mp3 --plan-only
# Fun-ASR 定时麦克风：录制 5 秒后上传
python3 main.py --microphone --record-seconds 5 --plan-only
# Fun-ASR 流式麦克风：实时显示 partial/final，回车提交
python3 main.py --microphone --stream --plan-only
# 原生 Omni 文件输入
python3 main.py --audio-backend omni --audio-file 夹起瓶子.mp3 --plan-only
# 原生 Omni Realtime 麦克风
python3 main.py --audio-backend omni-realtime --microphone --stream --plan-only
```

流式模式等待 `streaming now` 提示后说话，**回车提交，Ctrl+C 取消**，每次运行处理一条完整指令。Omni Realtime 持续上传音频，回车后才生成转写和计划，不显示 Fun-ASR 式逐字 partial；该后端的 `--stream` 可省略。

```bash
# 只转写：将上面音频调用中的 --plan-only 换成 --transcribe-only
python3 main.py --audio-backend omni-realtime --microphone --transcribe-only
# 查看设备后，用实际编号设置 --input-device-index
python3 main.py --list-input-devices
# 打印计划并进行 mock 工具执行
python3 main.py --audio-backend omni-realtime --microphone --print-plan
# 独立 Fun-ASR 转写模块
python3 audio_input.py --audio-file ../asr_test/Chinese.mp3
```

Omni 两种后端直接生成计划，省略 `--mock-planner`，无需 DeepSeek Key。`--transcribe-only` 不生成计划，标准输出只有最终文本。默认工具均为 mock，原有 `--real-*` 开关仍控制真实功能。

### 语音输入参数

| 参数 | 用途 / 默认值 |
|---|---|
| `--audio-backend` | `asr`、`omni`、`omni-realtime`，默认 `asr` |
| `--audio-file PATH` | 本地音频文件，与 `--microphone` 和位置参数文本指令互斥 |
| `--microphone` | 使用麦克风，与位置参数文本指令互斥 |
| `--stream` | ASR 边采集边识别，仅搭配麦克风；Omni Realtime 本身就是流式输入 |
| `--record-seconds N` | ASR 定时录音秒数，默认 5；不能与 `--stream` 或 Omni Realtime 同用 |
| `--input-device-index N` | PyAudio 输入设备编号，省略时使用默认设备 |
| `--list-input-devices` | 枚举输入设备后退出，无需 Key |
| `--transcribe-only` | 仅转写后退出，与 `--plan-only` 互斥 |
| `--plan-only` | 生成并打印计划，不执行工具，适用于全部后端 |
| `--print-plan` | 原有选项：打印计划后继续执行工具 |

### Fun-ASR 参数

| 参数 | 用途 / 默认值 |
|---|---|
| `--asr-model` | 默认 `AGENT_ASR_MODEL` 或 `fun-asr-realtime` |
| `--asr-workspace` | 覆盖 `DASHSCOPE_WORKSPACE` |
| `--asr-region` | 覆盖 `DASHSCOPE_REGION`，默认 `cn-beijing` |
| `--asr-sample-rate HZ` | 非 WAV 文件及麦克风的采样率，默认 16000；WAV 自动读取，显式指定时校验是否匹配 |
| `--asr-timeout SEC` | SDK 连接/读取超时、流式提交后的结果等待上限，默认 30 秒 |

ASR 后端沿用原文本 planner 的 `--model/--base-url/--api-key` 配置，也可加 `--mock-planner` 进行固定规则演示。

### Omni 参数

| 参数 | 用途 / 默认值 |
|---|---|
| `--omni-model` | HTTP 模型，默认 `AGENT_OMNI_MODEL` 或 `qwen3.8-omni-flash` |
| `--omni-base-url` | HTTP 接口，默认 `DASHSCOPE_BASE_URL`；未配置时按 Workspace / 地域构造 |
| `--omni-realtime-model` | Realtime 模型，默认 `AGENT_OMNI_REALTIME_MODEL` 或 `qwen3.8-omni-flash-realtime` |
| `--omni-ws-url` | WebSocket 接口，默认 `DASHSCOPE_REALTIME_URL`；未配置时按 Workspace / 地域构造，与 HTTP 地址独立 |
| `--omni-timeout SEC` | Omni 连接/读取超时及 Realtime 提交后的结果等待上限，默认 60 秒 |

`--model/--base-url/--api-key` 和 `--asr-*` 不配置 Omni。Realtime 执行失败后，复用 HTTP Omni，根据转写与执行历史重规划。

### 文件与输出约定

- ASR 支持 `.wav/.mp3/.pcm/.opus/.speex/.aac/.amr`；WAV 要求单声道 16-bit PCM，Opus/Speex 要求 Ogg 封装，AMR 要求 AMR-NB。采样率参数应与音频一致。
- HTTP Omni 支持 MP3 / PCM WAV，Base64 编码后小于 10 MB。
- Realtime 固定为 16 kHz、单声道、16-bit PCM；文件回放也只接受该规格的 WAV，至少 100 ms，不自动解码 MP3。
- ASR partial 仅用于展示，最终转写才进入 agent；Omni 计划通过本地校验后执行。
- HTTP Omni 的 `request_seconds` 为请求耗时；Realtime 的 `response_seconds` 从提交开始计时，不含录音和此前的上传时间。
- `[Done] plan-only; no tools executed` 表示只规划；执行模式下的 `success=True` 应结合工具的 mock / 真实状态判断。

### 语音模块离线测试

```bash
python3 -B -m unittest discover -s tests -v
```

测试使用模拟网络和麦克风，不消耗 API 额度，不控制机器人。

## 巡检 demo（输入"开始巡检"，小车依次到各房间）

固定巡检路线由 `planner.PATROL_ROUTE` 定义，各地点坐标在 `tools.LOCATION_COORDS` 配置。Mock 模式（无需 ROS，只打印计划）：

```bash
cd agent_demo
python3 main.py "开始巡检" --mock-planner --print-plan
```

真实导航模式（需已启动 base_robot + navigation 两个 launch）：

```bash
cd agent_demo
python3 main.py "开始巡检" --mock-planner --real-navigation
```

`--real-navigation` 通过 rclpy `ActionClient`（子进程，V1.5.0 `nav_waypoints.py` 同款路径）逐点真实导航 `NavigateToPose`。**不要用 `ros2 action send_goal` CLI**——本环境 FastDDS 下 goal response 发不回客户端，机器人不动但 CLI 干等（V2.0.0 联调实录）。默认 `ros_setup` 路径在 `tools.py` 里按项目根目录自动解析，通常无需 `--ros-setup`；若工作空间不在默认位置，可显式传入 `--ros-setup`。

## 颜色识别巡检（2026-10-28 已接入）

巡检时对每间房停靠位做一次颜色识别，按结果分支：

```text
for room in PATROL_ROUTE:
    navigate_to(room)
    color = detect_color(room).observations["color"]
    if color == "green":
        continue                            # 巡检下一个房间
    elif color == "red":
        navigate_to(f"{room}_table")        # 来到该房间的桌子面前
        vla_manipulate("pick up the cup and pour",   # 一整个 episode 走完
                       dock=room if 已标定工位)       # 抓杯→倾倒补药→放回→松爪
        continue                            # 巡检下一个房间
```

> 旧版"抓取杯子/放回杯子"两条中文指令是 OOD（v4 训练里没见过），V2.0.0 起红分支
> 只发**一条整段回合**；`dock` 给定时 `vla_manipulate` 先 gz teleport 精对齐底盘+杯
> 到验证工位（模型只在固定机位采过数据，Nav2 停靠 ±20cm 不够），当前仅 room2 已标定
> （`tools.DOCK_POSES`）。

实现方式：

- **真实检测**：`main.py --real-detection`（需仿真在跑）。`detect_color` 内部走两步子进程——
  1. 系统 python + source ROS 跑 `color_yolo/tools/capture_frames.py` 从前置相机抓 1 帧；
  2. conda `color` 环境跑 `color_yolo/tools/detect_onnx.py`（YOLO11n `best.onnx` @imgsz320，默认 conf=0.3，纯 CPU）。
  两个环境不能同进程（rclpy 在系统侧、ultralytics 在 conda 侧），所以拆成子进程串联。
- **红分支**：`executor.py` 检测到 `color=="red"` 时向原计划动态插入「去 `<room>_table` → 抓取杯子 → 放回杯子」子计划（navigate 带 verify_navigation、抓取带 verify_grasp）；green / none 不分支。
- **模型相关参数**：`--color-model`、`--color-python`、`--color-conf`；默认值在 `tools.py` 顶部常量。
- **停靠位**：四个房间停靠位在牌子正前方 **2.40m**（1.2m 时牌子出画，见顶层 README V1.8.0），与 `color_yolo/config/markers.yaml`、`nav_waypoints.py` 三处一致，改一处必须三处同步。
- 40 帧全量 ONNX 抽查 0 误检 0 漏检（conf=0.25），4 个停靠位联机实测颜色 4/4 正确。

```bash
# 真实导航 + 真实识别的完整巡检（仿真与 Nav2 需在跑；红色房会自动插抓杯子计划，
# 抓取本身还要 --real-vla-publish 才会真发到 /vla_instruction）
cd agent_demo
python3 main.py "开始巡检" --mock-planner --real-navigation --real-detection
```

## 四子系统整合 demo（V2.0.0：agent × Nav2 × color_yolo × OpenVLA）

一条命令跑通「来巡检点 → 认牌子 → 红色去桌前抓杯倾倒补药 → 继续巡检」：

```bash
cd agent_demo
bash run_room2_demo.sh          # room2（东南）单房巡检；日志 /tmp/demo_room2/
```

阶段：v4 推理服务器 → 仿真（launch 自启桥接，`vla_instruction` 默认 ''=topic 门控不抢跑）
→ 静置 75s 摆底盘/摆杯 → Nav2 栈 → agent（真 LLM 规划 + 真导航 + 真检测 + 真 VLA）。

**桥接集成协议**（V2.0.0 起，`vla_gazebo_bridge.py`）：

- `/vla_instruction` 任意非空消息 = 起新回合（同文案也重触发；全量重置回合状态）
- 回合起手 `auto_warm` 自动发 GT f1 预热位+张爪+静置（~15s，第十轮手工协议内置化）
- `/vla_episode_status`（String，1Hz）：`IDLE→WARMING→RUNNING→COMPLETE`

**VLA 生命周期（工位许可制）**——多房间巡检不为 7B 反复冷加载（重载 1~3 分钟/次）：

- 推理服务器开机加载后**全程常驻**（空闲零算力，只占显存）
- 每次 `navigate_to` 发放一次许可；`vla_manipulate` 消耗许可——同一停留点的重复
  操作步（如 LLM 多写的抓取）返回 success 并标注 `duplicate_skipped`，计划继续
- 回合超时/未成 COMPLETE → 许可回退，允许重试
- **整个计划收尾时 `main.py` 统一关停推理服务器**（打印 `[Cleanup]`）
- 单任务场景想 COMPLETE 即关服务器：`ToolRegistry(vla_shutdown_after_success=True)`

联调实录、验证结果与已知风险见 `test_records/2026-11-02_agent_patrol_room2_demo/`。

## 当前可用工具

所有工具都通过 `tools.py` 中的 `ToolRegistry` 注册。Executor 只会调用这些白名单工具。每个工具返回一个 `ToolResult`：

```python
ToolResult(
    tool="tool_name",
    success=True | False,
    message="human-readable summary",
    observations={...},
    suggested_recovery=[...],
)
```

当前工具：

| 工具 | 参数 | 当前行为 | 后续真实实现 |
|---|---|---|---|
| `navigate_to` | `location: str` | 默认 Mock 成功。`--real-navigation` 时通过 Nav2 `NavigateToPose` action 真实导航到 `tools.LOCATION_COORDS` 里配置的坐标。 | 已接 Nav2；后续可换 VLN service 或语义导航模块。 |
| `detect_object` | `object_name: str` | Mock 成功，置信度为 `0.9`。 | 替换为目标检测器 / VLM grounding，例如 YOLO、GroundingDINO、RGB-D 检测或场景图查询。 |
| `detect_color` | `location: str`（可选） | 默认 Mock 返回 `"green"`；`--real-detection` 时抓 1 帧相机图 → YOLO ONNX（CPU）识别，返回 `"red"` / `"green"` / `"none"` + 检测框。 | 已接摄像头 + YOLO11n；单次调用 ≈1.1s（进程启动为主），可换常驻节点降延迟。 |
| `vla_manipulate` | `instruction: str`，`dock: str?`，`wait_timeout: float?` | 默认 Mock 成功。`--real-vla-publish` 时：可选 gz teleport 精对齐 `DOCK_POSES[dock]` 工位 → 发布 `/vla_instruction` 起整段回合 → **阻塞轮询 `/vla_episode_status` 到 COMPLETE**（上限 `vla_wait_timeout`，默认 420s；防陈旧 COMPLETE：须先见 WARMING/RUNNING）。工位许可制去重，见下方整合 demo 一节。 | 桥接已上报回合状态；下一步可把 ROS 轮询换成 action/service 形态。 |
| `verify_navigation` | `location: str` | Mock 成功，`distance_to_goal=0.0`。 | 替换为 Nav2 action result 加 TF/AMCL 位姿阈值检查。 |
| `verify_grasp` | `object_name: str | None` | Mock 成功，`attached=True`。 | 替换为 Gazebo grasp plugin 事件、夹爪/接触传感器、力反馈或视觉确认。 |
| `verify_object_visible` | `object_name: str` | Mock 成功，`visible=True`。 | 替换为检测器/VLM 结果和置信度阈值判断。 |
| `verify_condition` | `condition: str` | 通用 Mock 成功。 | 替换为任务特定 verifier，例如物体位姿在目标区域、物体已释放、抽屉已打开、杯子在桌上。 |

plan step 示例结构：

```json
{
  "step_id": "3",
  "description": "Grasp the cup",
  "tool": "vla_manipulate",
  "args": {
    "instruction": "pick up the cup"
  },
  "verify": {
    "tool": "verify_grasp",
    "args": {
      "object_name": "cup"
    }
  }
}
```

重要约束：

- Planner 只能使用上面列出的工具。
- 工具参数必须匹配 `tools.py` 中 Python 方法的函数签名。
- 每个 plan step 的 `verify` 字段用于建模独立的验证工具调用。
- 当前验证工具都是占位实现。它们只能证明 `plan-execute` 控制流程可运行，不能证明真实机器人任务成功。

### 真实 VLA 发布路径

`vla_manipulate` 是当前唯一能产生真实外部副作用的工具。

默认 mock 行为：

```bash
python3 main.py "拿起前面的杯子" --mock-planner
```

真实发布行为：

```bash
python3 main.py "拿起前面的杯子" --mock-planner --real-vla-publish \
  --ros-setup $PROJECT_ROOT/base_robot_ws/install/setup.bash
```

这会发布到：

```text
/vla_instruction  std_msgs/msg/String
```

该 topic 由 `vla_gazebo_bridge.py` 消费。V2.0.0 起桥接会回报 `/vla_episode_status` 状态机，`vla_manipulate` 阻塞到 `COMPLETE` 才返回——能确认"整段回合执行完且臂已冻结"，但**杯是否真的倒净/放稳仍需真实 `verify_grasp` / `verify_condition`**（当前仍为 mock）。

## Verify 失败时的行为

验证发生在 `Executor.execute()` 内部，并且只会在当前 step 的主工具成功后执行。

每个 step 的执行顺序：

```text
1. call step.tool(args)
2. if tool result success == false:
     append failed StepRecord to AgentState.history
     stop current plan
     return (False, tool_result.message)
3. if step.verify exists:
     call verify.tool(verify.args)
4. if verify result success == false:
     mark StepRecord as failed
     attach verify_result to StepRecord
     append failed StepRecord to AgentState.history
     stop current plan
     return (False, verify_result.message)
5. otherwise append successful StepRecord and continue
```

因此，当某一步的 verify 失败时，executor 不会继续执行后续步骤。它会立即停止当前 plan，并将失败结果返回给 `main.py`。

`main.py` 随后执行如下逻辑：

```text
if execution failed and replans_used < max_replans:
    call Planner.replan(state, failure_message)
    execute the returned recovery plan
else:
    finish with success=False
```

失败的 tool result 和失败的 verify result 都会保存在 `AgentState.history` 中，因此 replanner 可以看到发生了什么。在真实 LLM 路径中，`Planner.replan()` 会把压缩后的执行历史发送给模型：

```json
[
  {
    "step_id": "3",
    "tool": "vla_manipulate",
    "status": "failed",
    "result": "Published VLA instruction to /vla_instruction: pick up the cup",
    "verify_result": "grasp verification failed"
  }
]
```

当前限制：mock planner 的 `replan()` 不会返回恢复计划。真实恢复行为需要使用 LLM planner 路径，并且需要真实 verifier 返回有意义的失败信息。后续可实现的恢复策略示例：

- `verify_navigation` 失败：重试导航、清理 costmap、选择备用 waypoint，或询问用户确认地点。
- `verify_object_visible` 失败：移动相机/底盘、在附近搜索，或询问用户澄清目标物体。
- `verify_grasp` 失败：重试抓取、调整接近位姿、打开夹爪后重新尝试，或询问用户。
- `verify_condition` 失败：执行任务特定恢复步骤，或安全停止。


## Example output

```text
[User] 去厨房拿一杯水回来
[Context] known_locations=['start', 'kitchen', 'living_room', 'table', 'user']
[Tools] ['detect_object', 'navigate_to', 'verify_condition', 'verify_grasp', 'verify_navigation', 'verify_object_visible', 'vla_manipulate']

[Plan]
[
  {
    "step_id": "1",
    "tool": "navigate_to",
    "args": {
      "location": "kitchen"
    },
    "verify": {
      "tool": "verify_navigation",
      "args": {
        "location": "kitchen"
      }
    },
    "description": "Navigate to the kitchen"
  },
  {
    "step_id": "2",
    "tool": "detect_object",
    "args": {
      "object_name": "glass of water"
    },
    "verify": {
      "tool": "verify_object_visible",
      "args": {
        "object_name": "glass of water"
      }
    },
    "description": "Locate a glass of water in the kitchen"
  },
  {
    "step_id": "3",
    "tool": "vla_manipulate",
    "args": {
      "instruction": "pick up the glass of water"
    },
    "verify": {
      "tool": "verify_grasp",
      "args": {
        "object_name": "glass of water"
      }
    },
    "description": "Grasp the glass of water"
  },
  {
    "step_id": "4",
    "tool": "navigate_to",
    "args": {
      "location": "user"
    },
    "verify": {
      "tool": "verify_navigation",
      "args": {
        "location": "user"
      }
    },
    "description": "Return to the user"
  }
]

[Execute] step=1 tool=navigate_to args={'location': 'kitchen'}
[Tool] success=True message=Mock navigation reached kitchen
[Verify] tool=verify_navigation args={'location': 'kitchen'}
[Verify] success=True message=Mock navigation verification passed for kitchen

[Execute] step=2 tool=detect_object args={'object_name': 'glass of water'}
[Tool] success=True message=Mock detector found glass of water
[Verify] tool=verify_object_visible args={'object_name': 'glass of water'}
[Verify] success=True message=Mock visibility verification passed for glass of water

[Execute] step=3 tool=vla_manipulate args={'instruction': 'pick up the glass of water'}
[Tool] success=True message=Mock VLA instruction accepted: pick up the glass of water
[Verify] tool=verify_grasp args={'object_name': 'glass of water'}
[Verify] success=True message=Mock grasp verification passed for glass of water

[Execute] step=4 tool=navigate_to args={'location': 'user'}
[Tool] success=True message=Mock navigation reached user
[Verify] tool=verify_navigation args={'location': 'user'}
[Verify] success=True message=Mock navigation verification passed for user

[Done] success=True message=plan completed
```

## 已知地点 grounding

将真实支持的地点传入上下文：

```bash
python3 main.py "去厨房拿一杯水回来" \
  --locations start,kitchen,living_room,table,user
```

Planner 会被要求只使用这些地点。后续该列表应来自地图配置或语义地图配置。

## 真实 VLA 发布

默认情况下所有工具都是 mock。若要将操作指令发布到 `/vla_instruction`（需仿真+桥接+推理服务器在跑）：

```bash
python3 main.py "拿起前面的杯子" --mock-planner --real-vla-publish \
  --ros-setup $PROJECT_ROOT/base_robot_ws/install/setup.bash
```

导航（`--real-navigation`）与颜色识别（`--real-detection`）已接真实实现；验证工具仍是 mock 占位。
全链路整合演示直接用 `run_room2_demo.sh`（见上文整合 demo 一节）。
