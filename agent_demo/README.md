# Agent Demo

该目录包含一个用于复杂机器人任务的最小 `plan-execute` agent 框架。

当前范围：

- 仅支持文本输入。
- 通过 OpenAI 兼容的 DeepSeek API 调用 LLM planner。
- 使用受限 JSON plan schema。
- Executor 只执行白名单工具。
- 导航、感知、验证工具当前均为 mock 实现。
- `vla_manipulate` 可选真实发布到 `/vla_instruction`。

## 使用 DeepSeek 兼容 API 运行

```bash
cd /root/openvla_new/VLA_ROBOT/agent_demo
export DEEPSEEK_API_KEY="your key"
python3 main.py "去厨房拿一杯水回来" --print-plan
```

默认 API 模板参考 `ds_demo.py`：

- `OpenAI(...)`
- `base_url="https://api.deepseek.com"`
- 默认模型：`deepseek-v4-pro`

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
| `navigate_to` | `location: str` | Mock 成功。返回 `observations.location` 和 `mode="mock"`。 | 替换为 Nav2 `NavigateToPose` action、VLN service 或语义导航模块。 |
| `detect_object` | `object_name: str` | Mock 成功，置信度为 `0.9`。 | 替换为目标检测器 / VLM grounding，例如 YOLO、GroundingDINO、RGB-D 检测或场景图查询。 |
| `vla_manipulate` | `instruction: str` | 默认 Mock 成功。使用 `--real-vla-publish` 时，会将 `instruction` 作为 `std_msgs/msg/String` 发布到 `/vla_instruction`。 | 保持为 OpenVLA bridge 入口，或后续替换为能报告完成状态的 ROS action/service。 |
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
  --ros-setup /root/openvla_new/VLA_ROBOT/base_robot_ws/install/setup.bash
```

这会发布到：

```text
/vla_instruction  std_msgs/msg/String
```

该 topic 由 `vla_gazebo_bridge.py` 消费。这只能确认指令已经发送到 VLA bridge，不能证明操作已经成功。要证明操作成功，需要实现真实的 `verify_grasp` / `verify_condition`。

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

默认情况下所有工具都是 mock。若要将操作指令发布到 `/vla_instruction`：

```bash
python3 main.py "拿起前面的杯子" --mock-planner --real-vla-publish \
  --ros-setup /root/openvla_new/VLA_ROBOT/base_robot_ws/install/setup.bash
```

导航、感知和验证当前仍是 mock 占位。后续需要将 `tools.py` 中的方法替换为 Nav2/VLN/感知/verifier 集成。
