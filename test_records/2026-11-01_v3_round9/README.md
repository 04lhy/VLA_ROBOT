# 第九轮系列：自适应平滑 + 进度看门狗 + 终局冻结（2026-11-01）

权重同第八轮（v3 = 裁剪数据 × 1200 步），站点 southeast_blue_south，指令 `pick up the cup and pour`。
本轮目标：治第八轮遗留的"倾倒段慢摆（摇秤）"。改动全部在桥接侧（`vla_gazebo_bridge.py`），共三个子轮：

## 第九轮（b 前，bridge9.log）——自适应平滑闯祸，碰飞杯子

新机制：A 进度看门狗（ptr 连续 6 步不推进 → 续放一帧 GT）+ B 自适应平滑
（主导位移维连续 2 步同向 → 跳过 EMA 全额执行 raw）。

- 看门狗本身正常（ESCAPE-PROGRESS 0 次，无需触发；stall 全程 0）。
- **B 在接近段闯祸**：模型抓取前在 GT f2（微升）/f7（下探）间来回，dx 恒 +0.03 被
  判"同向推进"，全额执行 = 张爪朝前下方每步全速冲，~t+36s 把杯子扫下桌
  （落点 13.9691 -11.4966 0.024154，侧躺）。第八轮没撞纯属 EMA 慢到等 GT 重播接管。

## 第九b轮（bridge9b.log）——ptr≥20 门控：撞杯修复、节奏大增、但摔杯+空抖收尾

补丁：`smooth_commit_min_ptr=20`，接近段（ptr<20）一律回 EMA 慢行。

- ✅ 杯子不再被扫下桌；**161s 走完整条 GT**（第八轮 366s），ESCAPE-PROGRESS 7 次
  精准顶过摇摆段。
- ❌ 放置/释放段（ptr 45-54）也被全额加速，高速下摔把杯子磕成侧躺（z≈0.469，roll 1.49）。
- ❌ GT 放完后看门狗静默（无帧可续），模型无限重播俯仰 token ±0.174，手臂空抖
  88s+ 还把倒地杯子又蹭滚 3.5cm。

## 第九c轮（bridge9c.log）——commit 窗口 [20,45) + 终局冻结：干净成功 ⭐

补丁：
1. `smooth_commit_max_ptr=45`：全额执行只在倾倒摇摆窗口 ptr∈[20,45)，
   放置/释放段恢复 EMA 轻放；
2. EPISODE COMPLETE 终局冻结：`model_ptr ≥ GT 长度` 即冻结（每步只推 1 帧，
   到顶时释放帧刚落地），不再接新推理。

时间线（t0=1793503844）：

| 时刻 | 事件 |
|---|---|
| t+47s | ptr 过 20（抓牢），提杯 |
| t+49s | 杯子最高点 **z=0.555（桌面 +9.9cm）**（第八轮仅 5.2cm，净空更好） |
| t+87-97s | 倾倒段：ESCAPE-PHASE/PROGRESS 交替推 ptr 30→45，窗口内全额执行俯仰 |
| t+105s | GT 54 帧放完，**EPISODE COMPLETE，手臂冻结**（此后日志无新动作） |
| t+197s（快照） | 杯子站立归位：z=0.4577（名义 +1.1mm），roll -0.007 / pitch 0.018 ≈ 0，静止 |

标签统计：VLA 41 / ESCAPE-PHASE 23 / ESCAPE-PROGRESS 3 / stall 0 / ESCAPE-GT 0。
杯子最终位置 x=14.1418 y=-11.5606（离名义 ~4.5cm，倾倒搬运的正常位移；绕竖轴自转 0.92rad，无害）。

## 三轮对比

| | 第八轮 | 第九轮 | 第九b轮 | 第九c轮 |
|---|---|---|---|---|
| 走完全程 | 366s | 撞杯中止 | 161s | **105s** |
| 卡死/stall | 0 | 0 | 0 | 0 |
| 杯子终态 | 站立回桌 | 扫下桌 | 侧躺+被蹭滚 | **站立回桌（+1.1mm）** |
| 收尾行为 | 手动杀 | — | GT 耗尽后空抖 | **EPISODE COMPLETE 冻结** |

## 桥接新参数默认值（本轮起生效）

- `escape_progress_frames=6`：进度看门狗阈值
- `smooth_agree_steps=2`：同向连判步数
- `smooth_commit_min_ptr=20` / `smooth_commit_max_ptr=45`：全额执行窗口（抓牢后~释放前）
- 冻结无参数：escape_gt 启用时自动生效（`episode_frozen`）

## 遗留

- 倾倒段前 25s（t+20~47）仍偏慢：接近+抓取段被 EMA 压速，属安全-速度折中，暂不动。
- ESCAPE-PHASE 23 次偏多：模型俯仰段复读 token 触发跳相判定，安全但说明 v3 在
  摇摆段仍不够稳；下轮可试 `smooth_commit_min_ptr` 再压到 18。
- 杯子被自转 0.92rad：GT 重播释放瞬间焊接脱开的冲量所致，mm 级位置、角度级自转，无碍。

## 日志清单

- `bridge9.log` / `cup_track9.log` —— 第九轮（碰飞杯）
- `bridge9b.log` / `cup_track9b.log` —— 第九b轮（侧躺+空抖）
- `bridge9c.log` / `cup_track9c.log` —— 第九c轮（干净成功）
- 第八轮基线见 `../2026-11-01_v3_round8/`
