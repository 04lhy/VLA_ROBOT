#!/bin/bash
# =====================================================================
# 单房间巡检整合演示（room2 东南）：全链路串起来跑一遍
#   Nav2 到停靠位 → YOLO 检测牌子 → 红色(本次已翻红) → agent 决策去桌前
#   → dock 精对齐 → OpenVLA v4 整段回合（抓杯→倾倒补药→放回） → 回中间。
# 前提：已 colcon build；权重 ~/models/gazebo_ft/gazebo_4room_v4；
#       conda color 环境 + color_yolo/model/best.onnx。
# 日志：/tmp/demo_room2/{deploy_server,sim,nav,agent}.log
# =====================================================================
# 注意：不能开 set -u —— ROS 的 setup.bash 里全是未定义变量引用
WS=/home/lhy/Desktop/VLA_ROBOT
LOG=/tmp/demo_room2
mkdir -p "$LOG"

say() { echo "[$(date +%H:%M:%S)] $*"; }
die() { say "FATAL: $*"; exit 1; }

# ---- 真收尾（2026-11-08 续测第5趟实锤修复）----
# 原脚本尾部那行"关仿真/导航"只是 say 打印，从不真杀：正常退出/FATAL 早退
# 都把 gzserver+桥+推理+tts+nav2 整栈留在机器上（第5趟 Step4 失败时机器
# 已 load 28，nav 等不到 odom TF 直接崩架；tts 跨 3 趟幸存）。下一趟 Step0
# 才擦屁股＝本趟死后机器持续被毒，链上趟趟互相拖累。trap 兜住所有退出路径。
teardown() {
  pkill -f "python3 -u main[.]py" 2>/dev/null || true
  pkill -f "deploy_4bit[.]py" 2>/dev/null || true
  pkill -f "ros2 launc[h]" 2>/dev/null || true
  pkill -f "gzclien[t]" 2>/dev/null || true
  pkill -f "gzserve[r]" 2>/dev/null || true
  pkill -f "component_container_isolate[d]" 2>/dev/null || true
  pkill -f "rvi[z]2" 2>/dev/null || true
  pkill -f "vla_gazebo_bridge[.]py" 2>/dev/null || true
  pkill -f "tts_node[.]py" 2>/dev/null || true
  pkill -f "robot_state_publishe[r]" 2>/dev/null || true
}
trap teardown EXIT
trap 'teardown; exit 143' TERM INT

# ---- 0. 清残留（括号模式防 pgrep/pkill 自匹配） ----
say "Step0 清理残留进程"
# 上一轮 agent 必须杀：它失败收尾时 tools.shutdown_vla_server() 会 pkill
# deploy_4bit——run16/17 两轮服务器都是在回合中被它杀掉的（strace si_pid 实测）。
pkill -f "python3 -u main[.]py" 2>/dev/null || true
pkill -f "deploy_4bit[.]py" 2>/dev/null || true
pkill -f "ros2 launc[h]" 2>/dev/null || true
pkill -f "gzclien[t]" 2>/dev/null || true
pkill -f "gzserve[r]" 2>/dev/null || true
pkill -f "component_container_isolate[d]" 2>/dev/null || true
pkill -f "rvi[z]2" 2>/dev/null || true
# [2026-11-07 实锤] vla_gazebo_bridge 必须单列清理：它由 launch 用独立会话拉起，
# 上一轮被外部掐停时 launch 死透、桥却幸存成孤儿——RUN1 实录 11 个同名桥同时在
# /vla_episode_status 上发陈旧冻结态，agent 单帧读状态永远撞上僵尸值，判"指令
# 未送达桥接"（其实本轮桥好好的）。杀父 launch 不连带，必须点名杀。
pkill -f "vla_gazebo_bridge[.]py" 2>/dev/null || true
# tts/robot_state_publisher 由 launch 拉起但 TERM 传播不到（续测第3/4趟各
# 漏一个 tts 幸存 45~85 分钟实锤），必须单列点名杀。
pkill -f "tts_node[.]py" 2>/dev/null || true
pkill -f "robot_state_publishe[r]" 2>/dev/null || true
# 等 strace/nav 等子进程真正退出再清 DDS 段：固定 sleep 不够，run17/19 实录
# 清段与旧 gzserver 退出重叠，新仿真 controller_manager 服务在 DDS 里不可见，
# spawner 空等 180s 判死。最多等 20s。
for _ in $(seq 1 20); do
  pgrep -f "gzserve[r]|ros2 launc[h]|component_container_isolat[e]" >/dev/null || break
  sleep 1
done
# SIGKILL 反复清杀会往 /dev/shm 漏 FastDDS 共享内存段——2026-11-04 彩排#3 实录：
# 新一轮进程能发现话题但数据通道错乱（/joint_states 有广告、样本全丢），
# 桥接永远等不到 "All joint states received"，Step2 干等 180s 判死。
# 上面进程全没了之后再扫段才安全（还活着的进程持有已打开段被删名会崩）。
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
sleep 2

source /opt/ros/humble/setup.bash
source "$WS/base_robot_ws/install/setup.bash"

# WSL 重启后的非登录 shell 里没有 DISPLAY：gzserver 的 OGRE 起不来
# （"Can't open display" → "Unable to create DepthCameraSensor. Rendering is
# disabled"），前置/腕部两个深度相机全部不存在，detect_color 抓不到帧、
# VLA 也没有图像输入（2026-11-04 实录）。WSLg 的 X0 socket 在就能直接回退。
if [ -z "$DISPLAY" ] && [ -S /tmp/.X11-unix/X0 ]; then
  export DISPLAY=:0
  say "DISPLAY 未设置，回退 :0（WSLg）"
fi
[ -n "$DISPLAY" ] || say "警告：仍无 DISPLAY，相机链路会挂"

# ---- 1. VLA 推理服务器（7B 加载要 1~3 分钟） ----
say "Step1 启动 v4 推理服务器"
# setsid：服务器独立会话——巡检脚本/工具 shell 的组信号波及不到它（run16 实录：
# 服务器在 Step3 被外部 SIGTERM 干掉，agent 白等一整轮）。strace 只跟信号，
# 若再被信号终止会记下 si_pid（发信号者），便于定位。
(cd "$WS/openvla/vla-scripts" && VLA_UNNORM_KEY= \
  setsid strace -f -qq -e trace=signal -o "$LOG/deploy_signals.log" \
  ~/miniforge3/envs/openvla_env/bin/python deploy_4bit.py \
  --openvla-path ~/models/gazebo_ft/gazebo_4room_v4 > "$LOG/deploy_server.log" 2>&1 &)
code=000
for _ in $(seq 1 60); do
  # 探活走 GET /latest-image，不要 POST /act -d '{}'（后者在服务器日志里刷 KeyError）
  code=$(curl -s -o /dev/null -w "%{http_code}" -m 3 http://127.0.0.1:8000/latest-image || true)
  [ "$code" = "200" ] && break
  sleep 5
done
[ "$code" = "200" ] || die "VLA 服务器没起来，见 $LOG/deploy_server.log"
say "VLA 服务器就绪"

# ---- 2. 仿真（launch 自启桥接：instruction='' topic 门控，不会抢跑） ----
# gui:=false：全程 headless。**绝不允许**改成"带 GUI 起、跑一半 pkill gzclient"
# ——2026-11-07 二分实锤：SIGTERM 已连接的 gzclient 会毒死 gzserver transport
# 入向消息泵，之后所有 gz model -p/-x（dock teleport/回读）挂死，
# agent 的 vla_manipulate 在 _snap_dock 处 3 轮超时、VLA 回合根本没起跑。
say "Step2 启动仿真（DEMO_GUI=${DEMO_GUI:-false}；true=带 gzclient 窗口给人看，**绝不能中途 pkill gzclient**——已连接客户端被杀会毒死 gzserver 传输泵）"
# [2026-11-07 三连 RUN2 实录] 冷启动竞态：gzserver 载世界时 load_controller 响应
# 超 10s 被 DDS 丢（launch 已把 spawner 超时放大到 30s 治本），但若仍撞车，
# arm_controller spawner FATAL → 就绪门永远等不到。整轮白等 180s 再 FATAL 太贵：
# 就绪轮询里同时盯 "Failed loading controller"，见到就提前进入**第 2 次整仿重开**
# （杀的是 launch/gzserver/桥，headless 无 gzclient，杀序无禁忌）。
sim_ready=""
for attempt in 1 2; do
  if [ "$attempt" = 2 ]; then
    say "Step2 第 2 次启动仿真（第 1 次未就绪，整仿清掉重开）"
    pkill -f "ros2 launc[h] base_robot" 2>/dev/null
    pkill -f "gzserve[r]" 2>/dev/null
    pkill -f "gzclien[t]" 2>/dev/null
    pkill -f "vla_gazebo_bridge[.]py" 2>/dev/null
    pkill -f "component_container_isolate[d]" 2>/dev/null
    sleep 6
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  fi
  if [ "${DEMO_GUI:-false}" = "true" ]; then
    # 带 GUI：按 launch 默认（历史实锤 -p gui 写法/中途杀 client 都是坑，别自创姿势）
    (DISPLAY=${DISPLAY:-:0} ros2 launch base_robot base_robot.launch.py > "$LOG/sim.log" 2>&1 &)
  else
    (ros2 launch base_robot base_robot.launch.py gui:=false > "$LOG/sim.log" 2>&1 &)
  fi
  ok=""
  for _ in $(seq 1 90); do
    grep -q "All joint states received" "$LOG/sim.log" && ok=1 && break
    # spawner 判死＝就绪门必挂，别空等满 180s，立刻进重开分支
    grep -q "Failed loading controller" "$LOG/sim.log" && break
    grep -qE "Traceback|SIGABRT" "$LOG/sim.log" && die "仿真启动异常，见 $LOG/sim.log"
    sleep 2
  done
  [ -n "$ok" ] && sim_ready=1 && break
done
[ -n "$sim_ready" ] || die "控制器/桥接两轮都没就绪，见 $LOG/sim.log"
say "桥接就绪：$(grep -m1 -o 'Grip close depth gate: allow z<=[0-9.]*' "$LOG/sim.log" || echo '无深度门控行?') / $(grep -m1 -o 'GT escape enabled.*' "$LOG/sim.log" | cut -c1-60)"

# ---- 3. 静置 75s 再摆位（V1.9.0 教训：世界新载立即 teleport 会被物理弹跳甩位） ----
say "Step3 静置 75s（物理沉降），然后底盘回起点 + 工位杯对齐"
sleep 75

# gz model 在 gzserver 没完全就绪时会**静默空返回**（2026-11-04 彩排#6 实录：
# 底盘/杯位读回全空=两条 teleport 根本没生效，工位杯差 8cm，VLA 抓取几何失效），
# 所以先探活，再传送，最后按坐标双测确认、不中就重试。
pose_at() { timeout 10 gz model -m "$1" -p 2>/dev/null; }
near() { awk -v a="$1" -v b="$2" -v t="$3" 'BEGIN{d=a-b; if(d<0)d=-d; exit !(d<=t)}'; }

bp=""
for _ in $(seq 1 20); do
  bp=$(pose_at base_robot)
  [ -n "$bp" ] && break
  sleep 4
done
[ -n "$bp" ] || die "gz model 一直空返回：gzserver 未就绪"

# 世界文件里 4 只杯本就各在一房桌上（四房采集布局），闲杯**不动**——
# 旧版脚本误信"杯叠原点"的陈年注释把它们挪到走廊堆着，既难看又没必要
# （底盘落点在 (0,0)，桌上杯根本碰不到，不存在碾杯 SIGABRT 场景）。
# 只把工位杯（clone）从 world 名义位对齐到 10q 标定位（差 ~8cm）。
cp=""
for _ in 1 2 3; do
  timeout 8 gz model -m base_robot -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0
  timeout 8 gz model -m plastic_cup_0_clone -x 14.152 -y -11.514 -z 0.456645 -R 0 -P 0 -Y 0
  sleep 8
  bp=$(pose_at base_robot); cp=$(pose_at plastic_cup_0_clone)
  read -r bx by _ <<<"$bp"
  read -r cx cy _ <<<"$cp"
  near "${bx:-9}" 0 0.05 && near "${by:-9}" 0 0.05 \
    && near "${cx:-9}" 14.152 0.02 && near "${cy:-9}" -11.514 0.02 && break
  say "摆位未确认（底盘=[$bp] 杯=[$cp]），重试"
done
say "底盘: $bp"
say "工位杯: $cp"
near "${cx:-9}" 14.152 0.02 && near "${cy:-9}" -11.514 0.02 \
  || die "工位杯没对齐到 10q 标定位：[$cp]"

# ---- 4. Nav2（map + AMCL + planner + RViz；set_initial_pose 在 15s 后发 (0,0,0)） ----
# [2026-11-07 三连重跑 RUN1 实录] 同款 DDS 冷启动丢响应轮到 nav2：
# "failed to send response to /nav2_container/_container/load_node (timeout)"
# → AMCL 组件链路断死、nav.log 里连一行 amcl 日志都没有 → map TF 永不存在。
# 与 spawner 不同，nav2 的 load_node 超时不在我们手里，但**整栈重拉**便宜且安全
# （只杀 nav 进程，不碰 gzserver——中毒禁忌只针对 gz 传输，重拉导航无此风险）。
say "Step4 启动导航栈"
nav_ready=""
for nav_try in 1 2; do
  if [ "$nav_try" = 2 ]; then
    say "Step4 第 2 次：清掉残废导航栈重拉（第 1 次等不到 map TF）"
    pkill -f "ros2 launc[h].*navigation" 2>/dev/null
    pkill -f "component_container_isolate[d]" 2>/dev/null
    pkill -f "rvi[z]2" 2>/dev/null
    sleep 5
  fi
  (ros2 launch base_robot navigation.launch.py > "$LOG/nav.log" 2>&1 &)
  # 就绪判据用 map→base_footprint TF，不是固定 sleep：AMCL 只有拿到 /scan 才会
  # 发布 map→odom，话题对不上时这条 TF 永远不出现，costmap 会刷满屏
  # "Invalid frame ID map"、Nav2 目标全被拒（2026-11-04 彩排#4 实录）。
  # 判"有没有输出行"而不是退出码：tf2_echo 拿到变换后仍持续打印、自己不会退，
  # 包 timeout 就恒返回 124，门控会一直空转到超时（彩排#6 的"卡住"假象）。
  tf_ok=""
  for _ in $(seq 1 30); do
    if timeout 10 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | grep -q "At time"; then
      tf_ok=1; break
    fi
    sleep 4
  done
  [ -n "$tf_ok" ] && nav_ready=1 && break
done
[ -n "$nav_ready" ] || die "两轮都等不到 map→base_footprint TF：AMCL 没发 map→odom，检查 /scan 与 nav.log，见 $LOG/nav.log"
say "导航栈就绪（map→base_footprint TF 可用）"

# （2026-11-07 删除）原"Step4 后 pkill gzclient 省渲染"的做法是本轮 dock 挂死的
# 真凶——中途杀已连接 GUI 会毒死 gzserver 入向 transport 泵，gz model 传送/回读
# 全部超时。现在 Step2 直接 gui:=false headless 起，GUI 从头就不存在，无需再杀。
# run16 的 GUI 渲染 abort 担忧由 headless 启动一并规避。想边跑边看：Step2 去掉
# gui:=false（注意：直接位置参数，ros2 launch 的 -p 是 --print-description），
# 但**不要**再往下面加回 pkill gzclient。

# ---- 5. agent：规划走真实 LLM（DeepSeek，key/模型见 agent_demo/.env：deepseek-flash）；
#        红/绿分支仍由 executor 拿实时 YOLO 结果决定。回退 mock：加 --mock-planner ----
say "Step5 agent 开跑：巡检 room2（真 LLM 规划+真导航+真检测+真 VLA）"
cd "$WS/agent_demo"
# -u：不缓冲 stdout——上一跑 agent 流水全压在管道缓冲里，进程被停就整段丢失
# --max-replans 3：dock teleport 之后 AMCL 需要几次激光才能重收 map→odom，
# 收粒前发目标会出现"None of the points of the global plan were in the local
# costmap"+Controller patience exceeded→ABORTED(status=6)。这是瞬时的，
# 重规划一次即通过（2026-11-04 实录：room3 段首尝试 ABORTED、重规划后成功；
# 默认预算 1 被这一段吃掉，最后回 home 段没预算直接判失败）。
# 完整版：四房全巡（顺序走圈 home→room1→room2→room3→room4→home）。
# 牌子现势：se(room2)=红 → 唯一触发 VLA 抓杯倾倒的站点；其余三房全绿直通
# （world diffuse 实锤，非工位房间不会撞上"未知工位"）。
# 四间同构厂房的 AMCL 镜像混淆无法根治（run3 实录 room4 段"原地不动却报
# 到达"），靠 tools.py 双保险兜住：gz 真值到点复核（NAV_GT_MISMATCH→重规划）
# + 发 goal 前 TF 新鲜度门控。预算按里程放大到 5（全程 ~140m，单次假到达
# 烧 1 个，run3/4 各烧 1-2 个）。
python3 -u main.py "依次巡检 room1、room2、room3、room4：每到一处先看牌子颜色，红色就抓杯倾倒补药，绿色就前往下一个房间，四房巡完后回 home" \
    --real-vla-publish --real-navigation --real-detection --max-replans 5 \
    2>&1 | tee "$LOG/agent.log"
rc=${PIPESTATUS[0]}

# 两条硬复核（比"VLA 回合 COMPLETE"更可信）：
#   桥接侧 PLACE-TAIL = 收口时有没有替模型把杯子送落桌面再松手；
#   agent 侧 工位杯复核 = 底盘离站后杯子是否真的还留在工位桌面上。
say "桥接放杯尾段: $(grep -o 'PLACE-TAIL:.*' "$LOG/sim.log" | tr '\n' ' ' || echo '未触发（模型自己走完了放杯+张爪）')"
say "工位杯复核: $(grep -o '工位杯复核：.*' "$LOG/agent.log" | tail -1 || echo '未执行（回合后没有再导航）')"

say "Demo 结束 rc=$rc（agent 收尾时已自动关停推理服务器）。日志目录：$LOG"
# 关仿真/导航/桥/tts：由脚本头部 trap teardown EXIT 真执行（FATAL 早退同样兜住），
# 不再是一行只会打印的 say。
