# -*- coding: utf-8 -*-
"""
本程序怎么解决两类跳变:

  (a) 断电重启跳变:
      退出时把当时的软件角度 last_q 记进配置文件。下次上电, 用本次读数
      raw(一定落在 0~56.85°)去推断"落在哪个候选零点", 让 q 和 last_q 连续;
      若发现差值是 LOBE 的整数倍, 就判定"认错了零点", 自动补 n×LOBE。

  (b) 断电后手动搬动跳变:
      手动把输出端搬过某个候选零点后再上电, 读数会"看起来正常但整体差 57°"。
      程序会把这种"和记忆不一致"的情况算出来并大声提示; 你可以用
      `set 0`(就地重设零点) / `lobe +1` / `shift` 直接把它改到正确位置。

关键点: 单圈编码器本身无法区分"差一个零点间距"的情况, 必须靠外部信息
(上次姿态 / 你手动确认的标记零点)才能确定, 这不是程序偷懒, 是物理限制。

----------------------------------------------------------------------
运行中在"等待输入"状态直接敲下面命令(回车确认):

  45              -> 缓慢转到输出端 45°  (键盘输入目标角度, 任务2/3)
  set [deg]       -> 把"当前物理位置"直接标定为 deg°(默认 0°)  ← 最直接的"修改零点"
  offset [deg]    -> 直接把零点偏移 offset 设成 deg°(高级)
  shift [deg]     -> 零点沿正方向平移 deg°(默认 30°)           ← 任务3 用
  lobe [+n/-n]    -> 零点整体平移 n 个候选零点间距(≈56.85°)    ← 修正"认错零点"
  home            -> 重新缓慢回到 0°
  damp            -> 进入阻尼模式(可自由拖动)
  status          -> 打印当前状态
  save            -> 手动保存配置
  help            -> 帮助
  quit / Ctrl-C   -> 阻尼退出

所有改动都会立即写入 motor_zero_cfg.json, 掉电不丢。
"""

import math
import time
import select
import sys
import json
import os

sys.path.append("../lib")
from unitree_actuator_sdk import *

# ------------------------------------------------------------------ 参数
SERIAL_NAME = "/dev/ttyUSB0"
MOTOR_TYPE  = MotorType.GO_M8010_6
MOTOR_ID    = 0
CFG_PATH    = "motor_zero_cfg.json"

N        = float(queryGearRatio(MOTOR_TYPE))   # 减速比 ≈ 6.33 (19:3)
TWO_PI   = 2.0 * math.pi
LOBE     = TWO_PI / N                          # 相邻候选零点间距(输出端) ≈ 0.9927 rad ≈ 56.85°

DT_TARGET   = 0.0002     # 主循环目标周期 200us
DT_MIN      = 1e-6
DT_MAX      = 0.002
MAX_SPEED   = 0.15       # 输出端插值最大角速度 rad/s (缓慢转动, 保护电机)
LIM         = 0.03       # 到位判定误差 rad (~1.7°)
STABLE_TIME = 0.2        # 连续稳定时间 s

KP_OUT      = 12.0       # 输出端位置刚度 (越大越硬)
KD_OUT      = 0.3        # 输出端阻尼系数
KD_DAMP     = 0.01       # 阻尼模式下转子侧阻尼

TEMP_WARN   = 80         # 温度提示阈值

HELP_TEXT = """
可用命令(在"等待输入"时直接敲, 回车确认):
  45            转到输出端 45°(键盘目标角度)
  set [deg]     把当前物理位置标定为 deg°(默认0°)  -- 直接修改零点
  offset [deg]  直接把零点偏移设成 deg°(高级)
  shift [deg]   零点沿正方向平移 deg°(默认30°)     -- 任务3: 零点正向偏移30度
  lobe [+n/-n]  零点整体平移 n 个零点间距(≈56.85°) -- 修正认错零点
  home          重新缓慢回到 0°
  damp          进入阻尼模式
  status        打印当前状态
  save          保存配置
  help          显示本帮助
  quit          退出(等同 Ctrl-C)
""".strip()


# ------------------------------------------------------------------ 小工具
def d2r(x):  #度->弧
    return x * math.pi / 180.0


def r2d(x):  #弧->度
    return x * 180.0 / math.pi


def step_toward(cur, tgt, step):   #限速插值:每次最多走 step,cur 逐渐逼近 tgt
    d = tgt - cur
    if d > step:
        return cur + step
    if d < -step:
        return cur - step
    return tgt


def load_cfg():
    cfg = {"zero_offset_rad": 0.0, "last_q_rad": None,
           "last_raw_rad": None, "gear_ratio": N}  #默认值
    try:
        with open(CFG_PATH, "r") as f:
            loaded = json.load(f)
        if isinstance(loaded, dict):
            cfg.update(loaded)   #用文件中的值覆盖cfg.
    except (FileNotFoundError, ValueError):
        pass
    return cfg


def save_cfg(cfg):
    tmp = CFG_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)  #先写临时文件
    os.replace(tmp, CFG_PATH)  #再原子替换


def read_line_nonblock():
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return None  #没输入返回 None(主循环继续跑),有输入才返回字符串
    return sys.stdin.readline().strip()  #strip()去除首尾空白


# ------------------------------------------------------------------ 主体
class MotorZero:
    def __init__(self):
        self.serial = SerialPort(SERIAL_NAME)
        self.cmd = MotorCmd()
        self.data = MotorData()
        self.cmd.motorType  = MOTOR_TYPE
        self.data.motorType = MOTOR_TYPE
        self.cmd.mode = queryMotorMode(MOTOR_TYPE, MotorMode.FOC) #FOC闭环模式
        self.cmd.id   = MOTOR_ID

        self.cfg = load_cfg()
        self.offset   = float(self.cfg["zero_offset_rad"])   # 输出端零点偏移
        self.last_q   = self.cfg["last_q_rad"]               # 上次退出的软件角度
        self.raw_norm = 0.0                                  # 上电读数归一化常数

        self.target     = 0.0        #目标位置（输出端）
        self.q_cmd      = 0.0        #当前下发位置（输出端）
        self.state      = "GO_ZERO"
        self.stable     = 0.0
        self.last_time  = time.perf_counter()  #上一帧时间戳
        self.last_print = self.last_time       #上一次打印时间

    # ---------------- 底层收发 ----------------
    def set_damping(self):
        self.cmd.tau = 0.0
        self.cmd.dq  = 0.0
        self.cmd.q   = 0.0
        self.cmd.kp  = 0.0
        self.cmd.kd  = KD_DAMP

    def damp(self, n=1):   #连发 n 帧阻尼,同时把最新反馈写进 self.data
        self.set_damping()
        for _ in range(n):
            self.serial.sendRecv(self.cmd, self.data)
            time.sleep(DT_TARGET)

    def send_pos(self):
        self.cmd.tau = 0.0
        self.cmd.dq  = 0.0
        self.cmd.kp  = KP_OUT / (N * N)
        self.cmd.kd  = KD_OUT / (N * N)
        self.cmd.q   = (self.q_cmd - self.offset) * N
        self.serial.sendRecv(self.cmd, self.data)

    # ---------------- 读数 ----------------
    def raw(self):
        """输出端原始读数 = 转子读数 / N (+ 上电归一化)。运行中它是连续值。"""
        return self.data.q / N + self.raw_norm

    def q_now(self):
        """软件看到的输出端角度。"""
        return self.raw() + self.offset

    def status_line(self):
        return ("state={:<11s} target={:8.2f} q_cmd={:8.2f} q_now={:8.2f} "
                "offset={:7.2f} temp={} err={}").format(
            self.state, r2d(self.target), r2d(self.q_cmd), r2d(self.q_now()),
            r2d(self.offset), self.data.temp, self.data.merror)

    # ---------------- 上电零点推断 ----------------
    def resolve_zero(self):
        self.damp(100)                     # 先阻尼, 拿到稳定的一帧反馈

        raw_meas = self.data.q / N
        self.raw_norm = -math.floor(raw_meas / LOBE) * LOBE   #相当于raw_meas对LOBE取余，然后乘以-LOBE，加在raw_meas上就是【0,LOBE】上的位置读数，正常情况下为 0
        raw0 = raw_meas + self.raw_norm                        # → 落在 [0, LOBE)
        base_offset = self.offset
        q_ref = raw0 + base_offset   #若本次和上次同一零点, 应读到的角度

        print("=" * 68)
        print("减速比 N = {:.4f},  相邻零点间距 LOBE = {:.2f}° (输出端)".format(N, r2d(LOBE)))
        print("本次上电原始读数  raw = {:8.2f}°   (理论上永远落在 0~{:.2f}°)".format(
            r2d(raw0), r2d(LOBE)))
        print("配置中的零点偏移 offset = {:8.2f}°".format(r2d(base_offset)))

        if self.last_q is None:
            self.offset = base_offset
            print("配置里没有上次姿态记录(首次运行)。先按当前 offset 使用;")
            print("建议把电机停在你标记的零点处, 再敲 `set 0` 做一次标定。")
            print("本次 offset = {:.2f}°,  当前 q = {:.2f}°".format(
                r2d(self.offset), r2d(self.q_now())))
            print("=" * 68)
            return

        diff  = self.last_q - q_ref   #只有当 raw0 是"一个周期内的余数"时,diff/LOBE 的整数部分才正好对应"跳了几个候选零点"。否则会整体偏移一个间距。
        n     = int(round(diff / LOBE))   #算出和上次位置相比差多少，需要补上几个零点位置
        resid = diff - n * LOBE           #补完之后剩下的误差，（|resid| ≤ LOBE/2）
        print("上次退出姿态 last_q = {:8.2f}°".format(r2d(self.last_q)))
        print("若沿用原零点, 本次应读数 = {:8.2f}°  -> 与记忆相差 {:8.2f}°".format(
            r2d(q_ref), r2d(diff)))
        print("           相差 = {:d}×LOBE {:+.2f}°".format(n, r2d(resid)))

        if n == 0:
            self.offset = base_offset   #没跳变，原样用
        else:
            print("-" * 68)
            print("!! 检测到零点可能跳变: 相差约 {:d} 个零点间距 ({:+.2f}°)。".format(
                n, r2d(n * LOBE)))
            print("   原因可能是: (a)这次上电姿态和上次不同; (b)断电后手动搬动过输出端。")
            print("   回车=采用记忆推断(自动补 n×LOBE)  +=调+1间距  -=调-1间距")
            print("   z=把当前位置直接设为0°(就地重标定)  k=不补偿")
            print("-" * 68)
            self.offset = self._confirm_jump(base_offset, raw0)   #跳变了，人工确认

        print("本次使用的 offset = {:8.2f}°,  当前 q = {:8.2f}°".format(
            r2d(self.offset), r2d(self.q_now())))
        print("=" * 68)

    def _print_jump_prompt(self, base_offset, n):
        """
        打印一次"补偿方案": 当前将显示的软件角度 + 可选操作。
        (以前是每个循环用 '\r' 重写同一行, 提示比终端宽会折行导致刷屏, 现改为只打印一次)

        参数:
            base_offset : 配置里保存的零点偏移(输出端, rad), 本次上电的基准值
            n           : 需要补偿的"候选零点间距"个数; 1 个 = LOBE ≈ 56.85°
        """
        # raw: 当前输出端原始读数。
        #      data.q 是转子侧角度 → 除以 N 换成输出端; raw_norm 是上电归一化常数(正常为 0)
        raw    = self.data.q / N + self.raw_norm
        # q_show: 补偿 n 个间距之后, 软件将要显示的输出端角度 = 读数 + (基准偏移 + n×间距)
        q_show = raw + base_offset + n * LOBE
        print("[补偿 n={:+d}] 当前将显示 q = {:8.2f}°".format(n, r2d(q_show)))
        print("  回车=确定   +=加1个间距   -=减1个间距   z=把当前位置设为0°   k=不补偿")

    def _confirm_jump(self, base_offset, raw0):
        """
        确认"零点跳变"要补偿多少个零点间距, 返回本次实际使用的 offset(输出端, rad)。

        背景(为什么要交互确认):
            单圈绝对值编码器每次上电只报 0~LOBE 的读数, "落在哪个候选零点之后"并不确定,
            于是整体角度可能差整数个 LOBE(≈56.85°)。若电机上电姿态和上次相差不大, 取
            "离上次姿态 last_q 最近的那个候选零点"即可自动纠正; 若相差很大(>半个间距),
            编码器本身无法区分, 必须由人在这里确认。

        参数:
            base_offset : 配置里保存的零点偏移(输出端, rad)
            raw0        : 本次上电的原始读数(raw() 归一化后, 落在 [0, LOBE))

        返回(最终 offset, 输出端 rad):
            回车/±      -> base_offset + n*LOBE   (采用记忆推断并补 n 个间距)
            k          -> base_offset             (不补偿, 沿用原 offset)
            z / set    -> -raw                    (把"当前物理位置"直接标定为 0°)
        """
        # n: 需要补偿的间距个数。用"本次读数"去对齐"上次记忆的姿态 last_q",
        #    差值除以 LOBE 后四舍五入取整 —— 即选择离 last_q 最近的那个候选零点
        n = int(round((self.last_q - (raw0 + base_offset)) / LOBE))
        self._print_jump_prompt(base_offset, n)          # 提示只打印一次
        while True:
            self.damp(1)                                 # 发一帧阻尼: 保持通信与读数刷新, 电机被"扶住"不乱跑
            line = read_line_nonblock()                  # 非阻塞读一行键盘输入; 没有输入时返回 None
            if line is None:                             # 还没有输入
                time.sleep(0.03)                         # 小睡一下再轮询, 避免空转占满 CPU
                continue
            low = line.strip().lower()
            # ---- 根据输入决定最终的 n / offset ----
            # 回车或 y/ok 都当作"确定", 采用记忆推断
            if low in ("", "y", "yes", "ok"):             # 确定: 补 n 个间距
                return base_offset + n * LOBE
            # +/- : 手动增减补偿的间距个数(半角/全角都收)
            if low in ("+", "＋"):                        # +1 个间距, 刷新提示后再让你确认
                n += 1
                self._print_jump_prompt(base_offset, n)
            elif low in ("-", "－", "—"):                 # -1 个间距
                n -= 1
                self._print_jump_prompt(base_offset, n)
            # k/no : 放弃补偿, 沿用原 offset
            elif low in ("k", "no"):                     # 不补偿: 沿用原 offset
                return base_offset
            # z/set : 把当前物理位置就地标定为 0°(重置零点)
            elif low in ("z", "set", "set0", "zero"):    # 就地重标定: 把"当前物理位置"当作 0°
                return -(self.data.q / N + self.raw_norm)
            # 其它输入: 回显你敲的内容 + 说明合法选项(方便排查)
            else:
                print("无效输入 {!r}: 这里只接受  回车=确定 / +=加1间距 / -=减1间距 / z=当前位置设为0° / k=不补偿".format(line))
                print("           (想输入目标角度的话: 先按 回车 结束确认, 回到正常模式再输入)")

    # ---------------- 命令解析 ----------------
    def persist(self):
        self.cfg["zero_offset_rad"] = self.offset
        self.cfg["gear_ratio"] = N
        save_cfg(self.cfg)

    def _hold_here(self):
        """零点的改动只改标签、不驱动机器人: 保持在当前物理位置。"""
        self.q_cmd  = self.q_now()
        self.target = self.q_now()
        self.stable = 0.0
        self.state  = "HOLD_TARGET"

    def handle_command(self, line):
        s   = line.strip()
        low = s.lower()
        if s == "":
            return
        if low in ("h", "help", "?"):
            print(HELP_TEXT)
            return
        if low.startswith("status") or low == "st":
            print(self.status_line())
            return
        if low == "home":
            self.q_cmd  = self.q_now()
            self.target = 0.0
            self.state  = "GO_ZERO"
            self.stable = 0.0
            print("-> 缓慢回到 0°")
            return
        if low in ("damp", "damping"):
            self.state = "DAMPING"
            print("-> 进入阻尼模式(可自由拖动)。敲角度或 home 恢复控制。")
            return
        if low in ("save",):
            self.persist()
            print("-> 配置已保存")
            return
        if low in ("quit", "exit"):
            raise KeyboardInterrupt

        if low.startswith("set"):
            parts = s.split()
            try:
                tgt = d2r(float(parts[1])) if len(parts) > 1 else 0.0
            except ValueError:
                print("用法: set [deg]")
                return
            self.offset += (tgt - self.q_now())     # 让当前物理位置显示 tgt
            self._hold_here()
            self.persist()
            print("-> 已把当前位置标定为 {:.2f}°;  新 offset = {:.2f}°".format(
                r2d(tgt), r2d(self.offset)))
            return

        if low.startswith("offset"):
            parts = s.split()
            try:
                val = d2r(float(parts[1])) if len(parts) > 1 else None
            except ValueError:
                val = None
            if val is None:
                print("用法: offset <deg>   当前 offset = {:.2f}°".format(r2d(self.offset)))
                return
            self.offset = val  #直接改偏移
            self._hold_here()  #保持在该状态
            self.persist()     #新的零点偏移保存在文件中
            print("-> offset 已直接设为 {:.2f}°;  当前 q = {:.2f}°".format(
                r2d(val), r2d(self.q_now())))
            return

        if low.startswith("shift"):
            parts = s.split()
            try:
                delta = d2r(float(parts[1])) if len(parts) > 1 else d2r(30.0)
            except ValueError:
                print("用法: shift [deg]")
                return
            self.offset -= delta                     # 零点正方向平移 delta
            self._hold_here()
            self.persist()
            print("-> 零点正向平移 {:.2f}°;  同一物理位置现在显示 {:.2f}°".format(
                r2d(delta), r2d(self.q_now())))
            return

        if low.startswith("lobe"):
            parts = s.split()
            try:
                n = int(parts[1]) if len(parts) > 1 else 1
            except ValueError:
                print("用法: lobe [+n/-n]")
                return
            self.offset += n * LOBE
            self._hold_here()
            self.persist()
            print("-> 零点平移 {:d} 个间距({:+.2f}°);  同一物理位置现在显示 {:.2f}°".format(
                n, r2d(n * LOBE), r2d(self.q_now())))
            return

        # 其余尝试当角度
        try:
            ang = float(s)  #当输入的是数字时，将其设为目标角度
        except ValueError:
            print("无法识别: {!r}  (输入 help 查看命令)".format(s))
            return
        if abs(ang) > 180.0:
            print("提示: 目标角 {:.2f}° 较大, 电机将缓慢转过去, 注意安全。".format(ang))
        self.q_cmd  = self.q_now()
        self.target = d2r(ang)
        self.state  = "GO_TARGET"
        self.stable = 0.0
        print("-> 目标角度 {:.2f}°".format(ang))

    # ---------------- 状态机 ----------------
    def _check_stable(self, dt, next_state, msg):
        if abs(self.q_cmd - self.target) < LIM and abs(self.q_now() - self.target) < LIM:
            self.stable += dt   # 命令和目标近, 实际和目标也近，加稳定时间
        else:
            self.stable = 0.0
        if self.stable > STABLE_TIME:
            self.state  = next_state
            self.stable = 0.0
            print(msg, flush=True)

    def run(self):   #主循环
        self.resolve_zero()
        self.q_cmd      = self.q_now()
        self.target     = 0.0
        self.state      = "GO_ZERO"
        self.stable     = 0.0
        self.last_time  = time.perf_counter()
        self.last_print = self.last_time
        print("开始: 缓慢回到 0° ...  (输入 help 查看命令)")

        try:
            while True:
                now = time.perf_counter()
                dt  = min(max(now - self.last_time, DT_MIN), DT_MAX)
                self.last_time = now

                line = read_line_nonblock()
                if line is not None:
                    self.handle_command(line)

                if self.state == "DAMPING":
                    self.set_damping()
                    self.serial.sendRecv(self.cmd, self.data)
                else:
                    if self.state == "GO_ZERO":
                        self.q_cmd = step_toward(self.q_cmd, self.target, MAX_SPEED * dt)   #插值限副，平滑靠近
                        self._check_stable(dt, "HOLD_ZERO",
                                           "-> 已回到 0°。请输入目标角度(度)回车: ")
                    elif self.state == "GO_TARGET":
                        self.q_cmd = step_toward(self.q_cmd, self.target, MAX_SPEED * dt)
                        self._check_stable(dt, "HOLD_TARGET",
                                           "-> 已到 {:.2f}°。继续输入目标角度: ".format(
                                               r2d(self.target)))
                    elif self.state == "HOLD_ZERO":
                        self.q_cmd = self.target
                    elif self.state == "HOLD_TARGET":
                        self.q_cmd = self.target
                    self.send_pos()           # 下发 (q_cmd - offset)*N

                if self.state not in ("HOLD_ZERO", "HOLD_TARGET") \
                        and now - self.last_print >= 0.3:
                    print(self.status_line())
                    self.last_print = now

                if self.data.merror != 0:
                    print("!! 电机报错 merror = {} (0正常 1过热 2过流 3过压 4编码器故障)".format(
                        self.data.merror))
                if self.data.temp is not None and self.data.temp > TEMP_WARN:
                    print("!! 电机温度较高: {}°C".format(self.data.temp))

                elapsed = time.perf_counter() - self.last_time
                if elapsed < DT_TARGET:
                    time.sleep(DT_TARGET - elapsed)

        except KeyboardInterrupt:
            print("\n用户中断。")
        finally:
            self.shutdown()

    def shutdown(self):
        self.damp(100)                                                   #先阻尼
        self.cfg["zero_offset_rad"] = self.offset                        #保存零点漂移
        self.cfg["last_q_rad"]   = self.q_now()                          #记录退出状态
        self.cfg["last_raw_rad"] = self.data.q / N + self.raw_norm
        self.cfg["gear_ratio"]   = N
        save_cfg(self.cfg)
        print("已进入阻尼, 配置已保存(last_q = {:.2f}°), 退出。".format(r2d(self.q_now())))


def main():
    try:
        app = MotorZero()
    except Exception as e:
        print("打开串口 {} 失败: {}".format(SERIAL_NAME, e))
        print("(确认已接 USB 转串口、已 sudo、端口号是否正确)")
        return
    app.run()


if __name__ == "__main__":
    main()
