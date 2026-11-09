#!/bin/bash
# =====================================================================
# 工位单回合回归：不起导航、不跑巡检，直接把车摆到 VLA 东南桌工位，起桥接、
# 发一条指令跑完整段回合，然后用**硬证据**判这次到底有没有夹住、有没有放好。
#
# 一次约 6~7 分钟（整趟巡检要 20 分钟）。调桥接门控 / 换权重都用它。
#   bash run_dock_vla_test.sh
#   GT_DIR=/tmp/gt_trunc45 bash run_dock_vla_test.sh    # 故障注入：切掉放杯尾段
#
# 判定口径（全部实测，不看 agent 那些 mock 验证器）：
#   夹住 = gzserver 日志 GazeboGraspFix: Attaching ≥1 次 且 杯底抬升 ≥3.5cm
#          （演示轨迹的物理抬升上限约 5cm，5cm 阈值会把执行略慢的回合误判为空抓）
#   松手 = GazeboGraspFix: Detaching ≥1 次，或回合末夹爪已张满
#   放好 = 末态杯子仍在工位名义位 ±30cm、z 在桌面高度带、roll/pitch 平坦
# 日志：/tmp/dock_test/{server,sim,bridge,cup_track,verdict}.log
# 退出码：0=三项全过；1=有没过（过在哪一项看 verdict.log）
# =====================================================================
# 注意：不能开 set -u——ROS 的 setup.bash 里全是未定义变量引用
WS=/home/lhy/Desktop/VLA_ROBOT
export LOG=/tmp/dock_test
GT_DIR=${GT_DIR:-$WS/openvla/datasets/gazebo_4room_v2/ep_0004}
# 工位=DOCK_POSES["room2"]（10q 标定值）：底盘 + 杯子
BASE_X=14.152; BASE_Y=-11.868; BASE_YAW=1.5708
CUP_MODEL=plastic_cup_0_clone; CUP_X=14.152; CUP_Y=-11.514; CUP_Z=0.456645
mkdir -p "$LOG"

say() { echo "[$(date +%H:%M:%S)] $*"; }
stop_tracker() { [ -n "${TRACKER_PID:-}" ] && kill "$TRACKER_PID" 2>/dev/null; }
die() { say "FATAL: $*"; stop_tracker; exit 1; }
trap stop_tracker EXIT

# ---- 0. 清场（括号模式防 pgrep/pkill 自匹配；进程都没了之后再扫 FastDDS 段） ----
say "Step0 清场 GT=$GT_DIR"
# 上一轮 agent 必须杀：它失败收尾时会 pkill deploy_4bit，把本轮服务器在回合中
# 杀掉（run16/17 实录，strace si_pid 抓到）。
pkill -f "python3 -u main[.]py" 2>/dev/null
pkill -f "deploy_4bi[t]" 2>/dev/null
pkill -f "ros2 launc[h]" 2>/dev/null
pkill -f "vla_gazebo_bridg[e]" 2>/dev/null
pkill -f "component_container_isolate[d]" 2>/dev/null
pkill -f "gzclien[t]" 2>/dev/null
pkill -f "gzserve[r]" 2>/dev/null
pkill -f "rvi[z]2" 2>/dev/null
# 等进程真正退出再清 /dev/shm：残留进程还持有 FastDDS 段时删名，新仿真会出现
# "Could not contact service /controller_manager/list_controllers"（run17/19 实录，
# 整场作废）。清段前最多等 20s。
for _ in $(seq 1 20); do
  pgrep -f "gzserve[r]|ros2 launc[h]|component_container_isolat[e]" >/dev/null || break
  sleep 1
done
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
sleep 2

source /opt/ros/humble/setup.bash
source "$WS/base_robot_ws/install/setup.bash"
# WSL 重启后的非登录 shell 里没有 DISPLAY：gzserver 的 OGRE 起不来 → 两个深度
# 相机全部不存在，桥接收不到画面（"Unable to create DepthCameraSensor"）
if [ -z "$DISPLAY" ] && [ -S /tmp/.X11-unix/X0 ]; then export DISPLAY=:0; fi
[ -n "$DISPLAY" ] || say "警告：仍无 DISPLAY，相机链路会挂"

# ---- 1. 推理服务器（7B 冷加载 20s~3min） ----
say "Step1 起 v4 推理服务器"
(cd "$WS/openvla/vla-scripts" && VLA_UNNORM_KEY= \
  ~/miniforge3/envs/openvla_env/bin/python deploy_4bit.py \
  --openvla-path ~/models/gazebo_ft/gazebo_4room_v4 > "$LOG/server.log" 2>&1 &)
code=000
for _ in $(seq 1 60); do
  # 探活走 GET /latest-image，不要 POST /act -d '{}'（后者在服务器日志里刷 KeyError）
  code=$(curl -s -o /dev/null -w "%{http_code}" -m 3 http://127.0.0.1:8000/latest-image || true)
  [ "$code" = "200" ] && break
  sleep 5
done
[ "$code" = "200" ] || die "推理服务器没起来，见 $LOG/server.log"
say "推理服务器就绪"

# ---- 2. 仿真 + 拦截自启桥接（要手动起桥接才能换 GT、看清门控日志） ----
# headless（gui:=false）：相机渲染在 gzserver 内、不需要 GUI；比"带 GUI 起再
# 中途 pkill"安全——SIGTERM 已连接的 gzclient 会毒死 gzserver transport 入向泵，
# 之后 gz model 传送/回读全挂（2026-11-07 run_room2_demo 实锤）。
say "Step2 起仿真（headless）并拦截自启桥接"
(ros2 launch base_robot base_robot.launch.py gui:=false > "$LOG/sim.log" 2>&1 &)
if timeout 240 bash -c 'while true; do
     if pgrep -f "vla_gazebo_bridge[.]py" >/dev/null; then
       # 等 1.5s 让它把 KDL/参数初始化完再杀：0.3s 就杀曾撞上 gzserver 插件
       # 初始化，gzserver 以 exit -6（boost shared_ptr 断言）直接崩掉（run8 实录）
       sleep 1.5; pkill -f "vla_gazebo_bridge[.]py"; echo INTERCEPTED; break
     fi; sleep 0.3
   done' | grep -q INTERCEPTED; then
  say "自启桥接已拦截"
else
  say "注意：没拦到自启桥接（可能启动慢或已退出）——见 $LOG/sim.log"
fi

# ---- 3. 等世界加载 + 静置 75s（V1.9.0 教训：新载物理弹跳会甩底盘） ----
say "Step3 等 /robot_description 到位后静置 75s"
timeout 300 bash -c 'until ros2 topic echo /robot_description --once --no-daemon \
    --field data >/dev/null 2>&1; do sleep 2; done' || die "拿不到 /robot_description"
# 控制器没激活就别往下跑：controller_manager 不可见时桥接/臂全是哑的，
# run17 实录白跑 8 分钟才在回合里发现。
timeout 120 bash -c "until grep -q 'Configured and activated arm_controller' '$LOG/sim.log' 2>/dev/null; do sleep 3; done" \
  || die "arm_controller 未激活（controller_manager 没起来），见 $LOG/sim.log"
sleep 75
timeout 20 ros2 topic echo /robot_description --once --no-daemon --field data \
  > "$LOG/rd_raw.txt" 2>/dev/null
python3 - "$LOG/rd_raw.txt" "$LOG/bridge_rd_params.yaml" <<'PY' || die "robot_description 转参数文件失败"
import sys, yaml
raw = open(sys.argv[1]).read()
i = raw.find('\n---\n')
s = raw[:i + 1] if i != -1 else raw
assert '<robot' in s, "读到的不是 urdf"
yaml.dump({'/**': {'ros__parameters': {'robot_description': s}}}, open(sys.argv[2], 'w'))
print("PARAMS_OK", len(s))
PY

# ---- 3b. 可选：先真实导航到桌前，复刻整套流程的底盘历史（NAV_FIRST=1） ----
# 工位单测是"传送后静置"，整套流程是"导航到位后再传送"：底盘可能带着残余俯仰/
# 被桌沿顶住，f8 关节位（相对底盘）就对不上杯子——run21 实录整套 attach=0、
# 而同样的桥接在单测里 attach=1。这个开关用来专门复现/验证该类几何差异。
if [ "${NAV_FIRST:-0}" = "1" ]; then
  say "Step3b 先导航到 room2 桌前（复刻整套流程的接近方式）"
  (ros2 launch base_robot navigation.launch.py > "$LOG/nav.log" 2>&1 &)
  tf_ok=""
  for _ in $(seq 1 40); do
    if timeout 10 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | grep -q "At time"; then
      tf_ok=1; break
    fi
    sleep 4
  done
  [ -n "$tf_ok" ] || die "导航栈 TF 不可用，见 $LOG/nav.log"
  timeout 150 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 14.15, y: -11.87, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7071081, w: 0.7071068}}}}" \
    --feedback > "$LOG/nav_goal.log" 2>&1 \
    && say "导航到位（room2 桌前）" \
    || say "导航未在大限内结束，继续（Step4 传送兜底）"
  # 传送前先停导航：send_goal 超时/取消后 DWB 还带残余速度，传送后会把底盘甩偏
  # （run13 实录：传送后 8s 偏 14cm、偏航 26°，臂直接顶不到 f8）。测试只要
  # 复刻"导航过"的底盘历史，不需要 Nav2 继续跑。
  pkill -f "navigation[.]launch[.]py" 2>/dev/null
  pkill -f "component_container_isolat[e]" 2>/dev/null
  pkill -f "rvi[z]2" 2>/dev/null
  sleep 4
fi

# ---- 4. 摆位到工位并双测确认（gz teleport 分轴参数，负坐标不能连写 -p） ----
say "Step4 摆位到东南桌工位"
# gz CLI 会返回 0 但静默不生效，必须回读确认、不中就重试（run15 实录 Step4
# 传送后底盘仍偏 43cm、杯子被撞倒）
near() { awk -v a="$1" -v b="$2" -v t="$3" 'BEGIN{d=a-b; if(d<0)d=-d; exit !(d<=t)}'; }
for _ in 1 2 3; do
  timeout 8 gz model -m base_robot -x "$BASE_X" -y "$BASE_Y" -z 0 -R 0 -P 0 -Y "$BASE_YAW"
  timeout 8 gz model -m "$CUP_MODEL" -x "$CUP_X" -y "$CUP_Y" -z "$CUP_Z" -R 0 -P 0 -Y 0
  sleep 8
  bp=$(timeout 6 gz model -m base_robot -p 2>/dev/null)
  cp=$(timeout 6 gz model -m "$CUP_MODEL" -p 2>/dev/null)
  read -r bx by _ <<<"$bp"; read -r cx cy cz _ <<<"$cp"
  near "${bx:-9}" "$BASE_X" 0.02 && near "${by:-9}" "$BASE_Y" 0.02 \
    && near "${cx:-9}" "$CUP_X" 0.02 && near "${cy:-9}" "$CUP_Y" 0.02 \
    && near "${cz:-9}" "$CUP_Z" 0.03 && break
  say "摆位未确认（底盘=[$bp] 杯=[$cp]），重试"
done
say "底盘: $bp"
say "工位杯: $cp"
near "${bx:-9}" "$BASE_X" 0.02 && near "${by:-9}" "$BASE_Y" 0.02 \
  || die "底盘没对齐到工位：[$bp]"

# ---- 5. 手动起桥接（10q 定案配方：topic 门控 + 自动预热 + 帧堆叠）+ 杯位记录 ----
say "Step5 起桥接与杯位记录器"
(ros2 run base_robot vla_gazebo_bridge.py --ros-args -p use_sim_time:=true \
  -p camera_topic:=/camera_sensor/image_raw -p position_scale:=1.0 -p rotation_scale:=1.0 \
  -p max_linear_step:=0.04 -p max_angular_step:=0.2 \
  -p escape_gt_file:="$GT_DIR/actions.npy" -p escape_gt_meta:="$GT_DIR/meta.json" \
  -p escape_stall_frames:=4 -p escape_phase_gap:=8 -p min_ee_z:=0.19 \
  -p stack_frames:=true --params-file "$LOG/bridge_rd_params.yaml" \
  > "$LOG/bridge.log" 2>&1 &)
# 记录器带退避：连续空读说明 gz master 正在被压垮（Gazebo Classic 11 长跑下
# CLI 查询通道会整体僵死，gz model 对任何模型都挂到超时），把间隔从 2s 拉到 10s，
# 别让记录器自己把通道打死——run5 实录整场 101 次空读、判定"无法判定"。
(while true; do
   line="$(timeout 5 gz model -m "$CUP_MODEL" -p 2>/dev/null)"
   if [ -z "$line" ]; then
     empties=$((empties + 1))
     if [ "$empties" -ge 3 ]; then sleep 10; else sleep 2; fi
   else
     empties=0; sleep 2
   fi
   echo "$(date +%s) $line"
 done > "$LOG/cup_track.log" 2>&1 &) &
TRACKER_PID=$!
timeout 150 bash -c 'until grep -q "All joint states received" "$LOG/bridge.log" 2>/dev/null
   do sleep 2; done' || die "桥接没就绪，见 $LOG/bridge.log"
sleep 3
grep -oE "GT escape enabled.*|GT joint trajectory loaded.*|Grip close depth gate.*" "$LOG/bridge.log"
[ "$(grep -c 'VLA action' "$LOG/bridge.log")" = "0" ] \
  || die "起手前就有 VLA action＝幽灵步，检查 instruction 参数"

# ---- 6. 发指令起回合，阻塞到 COMPLETE ----
# （2026-11-07 更新）原来这里"回合前 pkill gzclient"规避 run16 的 GUI abort——
# abort 与渲染队列问题已由 Step2 的 headless 启动（gui:=false）一并根治；
# 中途杀已连接 GUI 反而引入更严重的 transport 入向泵中毒（gz model 全挂）。
# 想边跑边看就去掉 Step2 的 gui:=false，但**绝不要再加回这里的 pkill**。
say "Step6 发指令起回合"
# 只有本回合内的附着事件才算数：拦截自启桥接时 gzserver 会遗留一对误附着/
# 脱开日志，旧判定把它算成 "attaching≥1"（run14 实录），掩盖了真实空抓。
EP_LINE=$(wc -l < "$LOG/sim.log")
timeout 15 ros2 topic pub --once /vla_instruction std_msgs/msg/String \
  "data: 'pick up the cup and pour'" >/dev/null 2>&1
done_ep=""
for _ in $(seq 1 100); do
  grep -q "EPISODE COMPLETE" "$LOG/bridge.log" && { done_ep=1; break; }
  grep -qE "Traceback" "$LOG/bridge.log" && die "桥接抛异常，见 $LOG/bridge.log"
  sleep 4
done
[ -n "$done_ep" ] || say "400s 内没等到 COMPLETE，按现有日志判定"
sleep 5
# 脱开物证探针：把臂移回预热位。杯子若还挂在/搁在爪上会被带着走，真松手的杯子
# 留在桌面高度带——这是"松手"唯一不可伪造的判据（dock run7：纸面已"张爪"，
# 实际杯被指尖托着 z=0.491，一移臂就掉下来）。
say "Step7 脱开探针（移开机械臂后判定）"
timeout 10 ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.0, 0.493, 0.084, 0.001, -0.5, -0.001], velocities: [0.0,0.0,0.0,0.0,0.0,0.0], time_from_start: {sec: 3}}]}" \
  >/dev/null 2>&1 || true
sleep 8
stop_tracker

# ---- 7. 硬证据判定 ----
python3 - "$LOG/cup_track.log" "$LOG/sim.log" "$LOG/bridge.log" \
         "$CUP_X" "$CUP_Y" "$CUP_Z" "$EP_LINE" <<'PY' | tee "$LOG/verdict.log"
import sys
track, simlog, blog, cx, cy, cz = (
    sys.argv[1], sys.argv[2], sys.argv[3], *(float(v) for v in sys.argv[4:7]))
start_line = int(sys.argv[7]) if len(sys.argv) > 7 else 0

rows = []
for line in open(track, encoding="utf-8", errors="ignore"):
    p = line.split()
    if len(p) >= 7:
        try:
            rows.append([float(v) for v in p[1:7]])
        except ValueError:
            pass

def count(path, needle, start=0):
    try:
        with open(path, encoding="utf-8", errors="ignore") as f:
            return sum(1 for i, l in enumerate(f) if i >= start and needle in l)
    except Exception:
        return 0

attaching = count(simlog, "GazeboGraspFix: Attaching", start_line)
detaching = count(simlog, "GazeboGraspFix: Detaching", start_line)
print(f"-- 回合步数={count(blog, 'VLA action')}  "
      f"插件 attaching={attaching} detaching={detaching}")
for line in [l.strip() for l in open(blog, encoding="utf-8", errors="ignore")
             if any(k in l for k in ("GRASP-ASSIST", "ARM-HOLD", "PLACE-TAIL",
                                     "RELEASE-CHECK", "EPISODE COMPLETE"))][-8:]:
    print("  ", line.split("]: ", 1)[-1])

if not rows:
    print("结论：杯位记录为空，无法判定（仿真/杯模型名不对？）")
    sys.exit(1)

maxz = max(r[2] for r in rows)
x, y, z, roll, pitch = rows[-1][0], rows[-1][1], rows[-1][2], rows[-1][3], rows[-1][4]
lift = maxz - cz
offset = ((x - cx) ** 2 + (y - cy) ** 2) ** 0.5
grasped = attaching >= 1 and lift >= 0.035
last_grip = [l for l in open(blog, encoding="utf-8", errors="ignore") if "Gripper:" in l]
last_grip = float(last_grip[-1].split("Gripper:")[-1]) if last_grip else -1
# 末态=**探针之后**的采样（脚本已在 COMPLETE 后把臂移走）：还挂在/搁在爪上的
# 杯子会被带走；真落桌的杯子留在真站立高度带。band 收紧到 0.436~0.478——
# 之前 0.40~0.60 把"被指尖托在 0.491"也判成了放好（dock run7 实录）。
standing = (0.436 <= z <= 0.478 and abs(roll) < 0.35 and abs(pitch) < 0.35)
released = standing
placed = standing and offset < 0.15
print("-- 探针：COMPLETE 后已把臂移回预热位，末态取探针后的采样")

print(f"-- 杯底抬升：最大 z={maxz:.4f}（名义 {cz:.4f}）→ {lift * 100:.1f}cm")
print(f"-- 末态：x={x:.3f} y={y:.3f} z={z:.4f} 偏离名义 {offset * 100:.1f}cm "
      f"roll={roll:.2f} pitch={pitch:.2f} 末帧夹爪开度={last_grip:.3f}")
print(f"结论：夹住={'是' if grasped else '否'}  松手={'是' if released else '否'}  "
      f"放好={'是' if placed else '否'}")
sys.exit(0 if (grasped and released and placed) else 1)
PY
# 注意用 PIPESTATUS：直接 $? 取到的是 tee 的退出码，判定失败也会被吞成 0
rc=${PIPESTATUS[0]}
say "判定 rc=$rc（明细见 $LOG/verdict.log；车/仿真还留着，下一步可人工看现场）"
exit $rc
