# -*- coding: utf-8 -*-
"""小车调参工具: 列出全部参数 -> 直接改 -> 一键编译烧录 (带身份核对)"""
import os, re, sys, json, threading, subprocess
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

BASE  = os.path.dirname(os.path.abspath(__file__))           # .../SmartCar/tools
PROJ  = os.path.dirname(BASE)                                 # .../SmartCar
F     = lambda rel: os.path.join(PROJ, rel)
ENV   = "bluepill_f103c8"
ELF   = os.path.join(PROJ, ".pio", "build", ENV, "firmware.elf").replace("\\", "/")  # openocd 只吃正斜杠
OPENOCD = os.path.expandvars(r"%USERPROFILE%\.platformio\packages\tool-openocd\bin\openocd.exe")
DEFAULTS_JSON = os.path.join(BASE, "参数默认值.json")

# (组名, [(定位串, 文件, 类型, 说明)])
# 类型: f=浮点(写回带f)  i=整数  coef=(Xf * PWM_MAX) 里的 X
GROUPS = [
 ("速度环 (轮速闭环)", "src/control.h", [
   ("#define SPD_KP ",    "f", "速度环比例 KP"),
   ("#define SPD_KI ",    "f", "速度环积分 KI"),
   ("#define SPD_KFF_L ", "f", "左轮前馈 (cm/s -> PWM)"),
   ("#define SPD_KFF_R ", "f", "右轮前馈"),
   ("#define SPD_ILIM ",  "f", "积分限幅"),
   ("#define SPD_LPF ",   "f", "速度滤波系数 0~1"),
   ("#define MIN_PWM ",   "f", "克服静摩擦最低 PWM"),
 ]),
 ("运动限制", "src/control.h", [
   ("#define ACC_LIMIT ",    "f", "加速度限幅 cm/s²"),
   ("#define SPD_MAX_CM_S ", "f", "目标速度上限 cm/s"),
   ("#define ROT_MAX_DPS ",  "f", "角速度上限 °/s"),
 ]),
 ("陀螺 / 几何", "src/board.h", [
   ("#define GYRO_SCALE ",     "f", "陀螺比例标定"),
   ("#define WHEEL_BASE_CM ",  "f", "轮距 cm"),
   ("#define ENCODER_CM_L ",   "f", "左轮编码器 每cm计数"),
   ("#define ENCODER_CM_R ",   "f", "右轮编码器 每cm计数"),
 ]),
 ("加速斜坡 / 轮速同步", "src/control.h", [
   ("#define ACC_SLEW_L ",          "f",    "循迹模式左轮斜坡"),
   ("#define ACC_SLEW_L_OTHER ",    "f",    "其它模式左轮斜坡"),
   ("#define KI_SYNC ",             "f",    "轮速同步积分增益 (0=关)"),
   ("#define SYNC_MAX ",            "f",    "轮速同步修正限幅 cm/s"),
 ]),
 ("摩擦补偿 / 加速", "src/control.c", [
   ("#define STICT_FF_L ",          "coef", "左轮低速摩擦补偿 (x PWM_MAX)"),
   ("#define STICT_FF_R ",          "coef", "右轮低速摩擦补偿"),
   ("#define START_ASSIST_MAX_L ",  "f",    "起步助推左限"),
   ("#define START_ASSIST_MAX_R ",  "f",    "起步助推右限"),
   ("#define STICT_FADE_CM_S ",     "f",    "补偿作用速度上限 cm/s"),
   ("#define SPIN_DUTY_MAX ",       "f",    "原地转 PWM 上限 (x PWM_MAX)"),
 ]),
 ("循线速度调度", "src/app.c", [
   ("static int p_cruise = ",           "i", "巡航速度 cm/s"),
   ("#define LINE_SEGMENT_CM ",         "f", "段长 cm (弯到弯)"),
   ("#define LINE_APPROACH_CM ",        "f", "减速点: 距下个弯 cm"),
   ("#define LINE_APPROACH_CM_S ",      "f", "进弯准备速度 cm/s"),
   ("#define LINE_LAUNCH_CM_S ",        "f", "首段起步速度 cm/s"),
   ("#define LINE_CRUISE_CM_S ",        "f", "循线基础速度 cm/s"),
 ]),
 ("直角弯执行", "src/app.c", [
   ("#define LINE_ARC_OUTER_CM_S ", "f", "弯道外轮速度 cm/s"),
   ("#define LINE_ARC_INNER_CM_S ", "f", "弯道内轮速度 (负=反转)"),
   ("#define LINE_TURN_DEG ",       "f", "转弯角度 度"),
   ("#define LINE_TURN_TRIM_C1 ",     "f", "第1弯角度微调 (正=多转)"),
   ("#define LINE_TURN_TRIM_C2 ",     "f", "第2弯角度微调"),
   ("#define LINE_TURN_TRIM_C3 ",     "f", "第3弯角度微调"),
   ("#define LINE_TURN_TRIM_C4 ",     "f", "第4弯角度微调"),
   ("#define LINE_TURN_TOL ",       "f", "到位容差 度"),
   ("#define LINE_PIVOT_DPS ",      "f", "原地转最高转速 °/s"),
   ("#define LINE_PIVOT_MIN_DEG ",  "f", "最小有效转角"),
   ("#define LINE_PIVOT_MAX_DEG ",  "f", "最大转角 (保护)"),
   ("#define CORNER_ADVANCE_CM_L ", "f", "左弯补偿前进 cm (偏前=调小)"),
   ("#define CORNER_ADVANCE_CM_R ", "f", "右弯补偿前进 cm (偏后=调大)"),
   ("#define CORNER_ADVANCE_CM_S ", "f", "补偿段速度 cm/s"),
   ("#define SPIN_GAIN_L ",         "f", "原地转左轮增益 (1.0=对称)"),
   ("#define LINE_SENSOR_OFFSET_CM ", "f", "探头中心相对车心横向偏移 cm"),
   ("#define LINE_SENSOR_PITCH_CM ",  "f", "探头横向间距 cm"),
 ]),
 ("出弯 / 穿越", "src/app.c", [
   ("#define LINE_PULL_CM ",          "f", "出弯直行距离 cm"),
   ("#define OBJ_TRACK_OFFSET_CM ", "f", "模式6: 传感器面到轨道的距离 cm (测距+1)"),
   ("#define OBJ_DETECT_CM ",       "f", "模式6: 检测物体距离 cm"),
   ("#define LINE_PULL_MAX_CM_S ",    "f", "出弯直行速度上限"),
   ("#define LINE_PULL_E_GIVEUP ",    "f", "偏差过大提前交回循线"),
   ("#define LINE_SETTLE_CREEP ",     "f", "对线爬行速度 cm/s"),
   ("#define LINE_SETTLE_MAX_CM ",    "f", "对线最大距离 cm"),
   ("#define LINE_SETTLE_TIMEOUT ",   "i", "对线超时 ms"),
   ("#define LINE_CROSSING_MAX_MS ",  "i", "穿越捕捉超时 ms"),
 ]),
 ("弯道检测 / 丢线", "src/app.c", [
   ("#define LINE_END_TURN_ENABLE ", "i", "单弯兜底开关 0关/1开"),
   ("#define LINE_END_TURN_CM ",     "f", "兜底触发距离 cm"),
   ("#define LINE_END_TURN_DIR ",    "i", "兜底转弯方向 -1左/1右"),
   ("#define LINE_GAP_MAX_CM ",      "f", "空白段最大 cm"),
   ("#define LOST_TIMEOUT_MS ",      "i", "丢线找线超时 ms"),
   ("#define LOST_ERR_MIN ",         "f", "丢线判弯阈值"),
   ("#define LINE_CORNER_DEG ",      "f", "计圈: 过弯航向阈值 度"),
   ("#define LINE_MIN_GAP_CM ",      "f", "计圈: 弯间最小距离 cm"),
 ]),
 ("循线纠偏 (画龙)", "src/app.c", [
   ("#define LINE_KP ", "f", "循线比例 KP"),
   ("#define LINE_KD ", "f", "循线阻尼 KD"),
   ("#define LINE_KGYRO ",        "f", "循线驼螺阻尼 (°/s反向, 0=关)"),
   ("#define LINE_KGYRO_ACC ",     "f", "加速瞬态额外驼螺阻尼"),
   ("#define LINE_KPSI ",         "f", "循线航向误差项 (主阻尼, 0=关)"),
   ("#define LINE_KEST ",         "f", "连续偏差估计修正权重"),
   ("#define LINE_DEADZONE ",     "f", "循线死区"),
   ("#define LINE_STRAIGHT_EMA ", "f", "直线航向平均系数"),
   ("#define LINE_STRAIGHT_MAX_RATE ", "f", "学线方向时的最大转速 (°/s)"),
   ("#define LINE_PIVOT_LEAD_TRIM ",  "f", "原地转刹车提前量补正 (度)"),
 ]),
 ("航向环 (空白段/出弯)", "src/app.c", [
   ("#define HEAD_KP ", "f", "航向比例 KP (大了会左右摆)"),
   ("#define HEAD_KD ", "f", "航向阻尼 KD (小了会左右摆)"),
   ("#define HEAD_KI ",            "f", "航向环积分 (消空白段常值漂移)"),
   ("#define HEAD_ILIM ",          "f", "航向环积分限幅"),
 ]),
 ("模式二 转向", "src/app.c", [
   ("#define TURN_TOL_DEG ",     "f", "到位容差 度"),
   ("#define TURN_BRAKE_DEG ",   "f", "刹车进入角 度"),
   ("#define TURN_MAX_DPS ",     "f", "最大转速 °/s"),
   ("#define TURN_TRIM_DPS ",    "f", "微调转速 °/s"),
   ("#define TURN_MIN_DPS ",     "f", "最低转速 °/s"),
   ("#define TURN_ACCEL_DPS2 ",  "f", "转速加速度 °/s²"),
   ("#define TURN_BRAKE_DPS2 ",  "f", "刹车预测减速度"),
   ("#define TURN_STOP_DPS ",    "f", "停转判据 °/s"),
   ("#define TURN_SETTLE_MS ",   "i", "刹车后稳定 ms"),
   ("#define TURN_MAX_TRIMS ",   "i", "微调次数上限"),
 ]),
 ("模式三 里程", "src/app.c", [
   ("#define DIST_TOL_CM ", "f", "停车容差 cm"),
   ("#define DIST_DECEL ",  "f", "停车减速度 cm/s²"),
 ]),
 ("模式七 遥控", "src/app.c", [
   ("#define RC_TIMEOUT_MS ", "i", "断联刹车 ms"),
   ("#define RC_LIN_CM_S ",   "f", "满杆前进速度 cm/s"),
   ("#define RC_ROT_DPS ",    "f", "满杆转速 °/s"),
 ]),
]

def read_val(path, anchor, typ):
    """返回 (值 or None, 整行文本)"""
    try:
        txt = open(F(path), encoding="utf-8").read()
    except OSError:
        return None, ""
    i = txt.find(anchor)
    if i < 0:
        return None, ""
    j = txt.find("\n", i)
    line = txt[i:j if j >= 0 else len(txt)]
    m = re.search(r"[-+]?\d+(?:\.\d+)?", line[len(anchor):])
    return (float(m.group(0)) if m else None), line

def fmt_val(typ, v):
    if typ == "f":
        t = "%g" % v
        if "." not in t and "e" not in t:
            t += ".0"          # 整数浮点保留 .0, 原值写回不破坏文本
        return t + "f"
    if typ == "coef":
        return ("(%gf * PWM_MAX)" % v)
    return str(int(round(v)))

def write_val(path, anchor, typ, val):
    p = F(path)
    txt = open(p, encoding="utf-8").read()
    i = txt.find(anchor)
    if i < 0:
        raise RuntimeError("找不到 " + anchor)
    j = txt.find("\n", i)
    k = txt.rfind("\n", 0, i) + 1
    line = txt[k:j if j >= 0 else len(txt)]
    if typ == "coef":
        # coef 类型: 整段 (Xf * PWM_MAX) 一起换, 否则会嵌套出 ((Xf*PWM)f*PWM)
        m = re.search(r"\([-+]?\d+(?:\.\d+)?f? \* PWM_MAX\)", line)
        if not m:
            raise RuntimeError("coef 表达式没找到: " + anchor)
        a = k + m.start()
        b = k + m.end()
    else:
        m = re.search(r"[-+]?\d+(?:\.\d+)?f?", line[len(anchor):])
        if not m:
            raise RuntimeError("定位串后没有数值: " + anchor)
        a = k + len(anchor) + m.start()
        b = a + m.end() - m.start()
    txt = txt[:a] + fmt_val(typ, val) + txt[b:]
    open(p, "w", encoding="utf-8").write(txt)

class Tool:
    def __init__(self, root):
        self.root = root
        root.title("小车调参工具 — 改完点【应用修改】, 再点【编译+烧录】")
        root.geometry("760x680")
        top = ttk.Frame(root); top.pack(fill="x", padx=6, pady=4)
        for txt, cmd in (("应用修改", self.apply), ("编译", self.build),
                         ("烧录", self.flash), ("编译+烧录", self.build_flash),
                         ("恢复默认", self.restore), ("保存配置", self.save_cfg),
                         ("加载配置", self.load_cfg)):
            ttk.Button(top, text=txt, command=cmd).pack(side="left", padx=3)
        self.status = ttk.Label(top, text=""); self.status.pack(side="right")

        wrap = ttk.Frame(root); wrap.pack(fill="both", expand=True, padx=6)
        canvas = tk.Canvas(wrap, highlightthickness=0)
        sb = ttk.Scrollbar(wrap, orient="vertical", command=canvas.yview)
        self.frame = ttk.Frame(canvas)
        self.frame.bind("<Configure>",
                        lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.frame, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        canvas.bind_all("<MouseWheel>",
                        lambda e: canvas.yview_scroll(-e.delta // 120, "units"))

        self.entries = {}
        for gname, path, items in GROUPS:
            ttk.Label(self.frame, text="◆ " + gname,
                      font=("", 10, "bold")).pack(anchor="w", pady=(8, 1))
            for anchor, typ, desc in items:
                row = ttk.Frame(self.frame); row.pack(fill="x", padx=14, pady=1)
                ttk.Label(row, text=desc, width=30).pack(side="left")
                val, _ = read_val(path, anchor, typ)
                var = tk.StringVar(value=("?" if val is None else
                                          ("%g" % val if typ != "i" else str(int(val)))))
                ent = ttk.Entry(row, width=10, textvariable=var); ent.pack(side="left", padx=4)
                self.entries[(gname, anchor, path, typ)] = var
                ttk.Label(row, text=anchor.strip("# ").split()[0] if anchor.startswith("#")
                          else "p_cruise", width=20,
                          foreground="#888").pack(side="left")

        self.logbox = scrolledtext.ScrolledText(root, height=10, font=("Consolas", 9))
        self.logbox.pack(fill="both", padx=6, pady=4)
        self.logbox.insert("end", "改参数 -> 应用修改 -> 编译+烧录 (车通电、ST-Link 接好)\n")
        self.capture_defaults()
        self.log("就绪")

    def log(self, msg):
        self.logbox.insert("end", msg + "\n"); self.logbox.see("end")

    def collect(self):
        vals = {}
        for (gname, anchor, path, typ), var in self.entries.items():
            raw = var.get().strip()
            if raw in ("", "?"):
                continue
            try:
                vals[(gname, anchor, path, typ)] = float(raw)
            except ValueError:
                raise RuntimeError("参数不是数字: " + desc_of(gname, anchor))
        return vals

    def apply(self):
        try:
            vals = self.collect()
        except RuntimeError as e:
            messagebox.showerror("错误", str(e)); return
        errs = []
        for (gname, anchor, path, typ), v in vals.items():
            try:
                write_val(path, anchor, typ, v)
            except Exception as e:
                errs.append("%s: %s" % (anchor.strip(), e))
        if errs:
            messagebox.showerror("部分失败", "\n".join(errs))
        else:
            self.log("✔ 已写入 %d 个参数" % len(vals))

    def capture_defaults(self):
        if os.path.exists(DEFAULTS_JSON):
            return
        snap = {}
        for (gname, anchor, path, typ) in self.entries:
            v, _ = read_val(path, anchor, typ)
            if v is not None:
                snap["%s|%s|%s|%s" % (gname, anchor, path, typ)] = v
        json.dump(snap, open(DEFAULTS_JSON, "w", encoding="utf-8"),
                  ensure_ascii=False, indent=1)
        self.log("已记录当前值为默认值: " + DEFAULTS_JSON)

    def restore(self):
        if not os.path.exists(DEFAULTS_JSON):
            messagebox.showinfo("提示", "没有默认值记录"); return
        snap = json.load(open(DEFAULTS_JSON, encoding="utf-8"))
        errs = []
        for key, v in snap.items():
            gname, anchor, path, typ = key.split("|")
            try:
                write_val(path, anchor, typ, v)
            except Exception as e:
                errs.append(str(e))
        for (gname, anchor, path, typ), var in self.entries.items():
            v, _ = read_val(path, anchor, typ)
            if v is not None:
                var.set("%g" % v if typ != "i" else str(int(v)))
        self.log("✔ 已恢复默认" + (" (部分失败)" if errs else ""))

    def save_cfg(self):
        p = os.path.join(BASE, "调参配置_" +
                         __import__("time").strftime("%m%d_%H%M") + ".json")
        json.dump({k: var.get() for k, var in
                   [("%s|%s" % (g, a), var) for (g, a, pp, t), var in self.entries.items()]},
                  open(p, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        self.log("✔ 配置已存: " + p)

    def load_cfg(self):
        p = filedialog.askopenfilename(initialdir=BASE, filetypes=[("JSON", "*.json")])
        if not p: return
        cfg = json.load(open(p, encoding="utf-8"))
        for (gname, anchor, path, typ), var in self.entries.items():
            key = "%s|%s" % (gname, anchor)
            if key in cfg:
                var.set(cfg[key])
        self.log("✔ 配置已载入 (记得点 应用修改)")

    # ---- 编译 / 烧录 ----
    def run_stream(self, cmds, done_msg):
        def work():
            for cmd in cmds:
                p = subprocess.Popen(cmd, cwd=PROJ,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     creationflags=0x08000000)
                for line in iter(p.stdout.readline, b""):
                    self.root.after(0, self.log, line.decode("utf-8", "ignore").rstrip())
                p.wait()
                if p.returncode != 0:
                    self.root.after(0, self.log, "✘ 失败 (返回码 %d)" % p.returncode)
                    self.root.after(0, self.set_status, "失败")
                    return
            self.root.after(0, self.log, done_msg)
            self.root.after(0, self.set_status, "完成")
        threading.Thread(target=work, daemon=True).start()

    def set_status(self, s):
        self.status.config(text=s)

    def build(self):
        self.set_status("编译中...")
        self.run_stream([[sys.executable, "-m", "platformio", "run", "-e", ENV]],
                        "✔ 编译完成")

    def _check_target(self):
        """读芯片全片核对身份标识。返回 True=可以烧, False=拒绝/失败。"""
        oc = OPENOCD
        chk = [oc, "-f", "interface/stlink.cfg", "-f", "target/stm32f1x.cfg",
               "-c", "init",
               "-c", "dump_image " + os.path.join(BASE, "target_chip.bin").replace("\\", "/") + " 0x08000000 0x10000",
               "-c", "shutdown"]
        try:
            subprocess.run(chk, cwd=PROJ, capture_output=True,
                           creationflags=0x08000000)
            blob = open(os.path.join(BASE, "target_chip.bin"), "rb").read()
            if b"REMOTE-01" in blob:
                messagebox.showerror("拒绝烧录", "当前接的是【遥控器】不是小车!")
                self.set_status("已拒绝"); return False
            if b"CAR-01" not in blob:
                self.log("⚠ 芯片里没有小车标识, 请确认接的是小车")
            return True
        except Exception as e:
            messagebox.showerror("错误", "ST-Link 读取失败:\n" + str(e))
            self.set_status("失败"); return False

    def flash(self):
        oc = OPENOCD
        if not os.path.exists(oc):
            messagebox.showerror("错误", "找不到 openocd:\n" + oc); return
        self.set_status("烧录中...")
        if not self._check_target():
            return
        prog = [oc, "-f", "interface/stlink.cfg", "-f", "target/stm32f1x.cfg",
                "-c", "program " + ELF + " verify", "-c", "reset run", "-c", "shutdown"]
        self.run_stream([prog], "✔ 烧录完成, 车已复位")

    def build_flash(self):
        self.apply()                      # 先把参数写进源码
        self.set_status("编译+烧录...")
        oc = OPENOCD
        if not os.path.exists(oc):
            messagebox.showerror("错误", "找不到 openocd:\n" + oc); return
        if not self._check_target():      # 编译+烧录同样核对身份
            return
        cmds = [[sys.executable, "-m", "platformio", "run", "-e", ENV],
                [oc, "-f", "interface/stlink.cfg", "-f", "target/stm32f1x.cfg",
                 "-c", "program " + ELF + " verify", "-c", "reset run", "-c", "shutdown"]]
        self.run_stream(cmds, "✔ 编译+烧录完成, 车已复位")

def desc_of(gname, anchor):
    for g, path, items in GROUPS:
        if g == gname:
            for a, t, d in items:
                if a == anchor:
                    return d
    return anchor

def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("vista")
    except Exception:
        pass
    Tool(root)
    root.mainloop()

if __name__ == "__main__":
    main()
