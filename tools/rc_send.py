"""模式七 蓝牙遥控 —— 电脑端控制器 (WASD)

架构: 电脑 --(蓝牙)--> HC-06 --(串口)--> 小车固件(模式七)

用法:
    双击  遥控启动.bat          (推荐, 自动找蓝牙串口)
    python rc_send.py              不带参数 = 自动找蓝牙串口
    python rc_send.py COM5         手动指定
    python rc_send.py COM5 --baud 115200

操作 (真·按住才走, 松手即停):
    按住 w = 前进    s = 后退    a = 左转    d = 右转
    w+a / w+d / s+a ... 可以组合 = 边走边转 (混控帧)
    方向键 上/下/左/右 也可以
    x 或 空格 = 停
    + / - = 调整力度 (默认 40, 范围 10~100)
    q = 退出 (退出前自动发停止)
"""
import sys, time
try:
    import serial
except ImportError:
    raise SystemExit("需要 pyserial:  pip install pyserial")
try:
    import ctypes
    _user32 = ctypes.windll.user32   # 系统级按键状态, 能区分"按住"和"松开"
except (ImportError, AttributeError):
    raise SystemExit("这个脚本目前只支持在 Windows 上运行")

VK = {"w": 0x57, "s": 0x53, "a": 0x41, "d": 0x44, "x": 0x58, "q": 0x51,
      "up": 0x26, "down": 0x28, "left": 0x25, "right": 0x27,
      "space": 0x20,
      "plus": 0xBB, "minus": 0xBD, "numpad_plus": 0x6B, "numpad_minus": 0x6D}


def key_down(name):
    return bool(_user32.GetAsyncKeyState(VK[name]) & 0x8000)

CMD = {"w": 0x01, "s": 0x02, "a": 0x03, "d": 0x04, "x": 0x05, " ": 0x05}
NAME = {0x01: "前进", 0x02: "后退", 0x03: "左转", 0x04: "右转", 0x05: "停止"}
RESEND = 0.15            # 每 150ms 重发一次当前指令 (固件 500ms 断联停车)


def frame(cmd, arg):
    b = [0xAA, 0x55, cmd & 0xFF, arg & 0xFF]
    b.append(b[0] ^ b[1] ^ b[2] ^ b[3])
    return bytes(b)


def mix_frame(lin, rot):
    """混控帧: 前进和转向同时给 (固件命令字 0x06), LIN/ROT 范围 -100~100"""
    b = [0xAA, 0x55, 0x06, lin & 0xFF, rot & 0xFF]
    b.append(b[0] ^ b[1] ^ b[2] ^ b[3] ^ b[4])
    return bytes(b)


def describe(lin, rot):
    p = []
    if lin > 0:   p.append("前进")
    elif lin < 0: p.append("后退")
    if rot > 0:   p.append("右转")
    elif rot < 0: p.append("左转")
    return "+".join(p) if p else "停止"


def send(ser, cmd, arg):
    f = frame(cmd, arg)
    ser.write(f)
    ser.flush()


CFG = __file__.rsplit(".", 1)[0] + "_port.txt"   # 记住上次用的串口


def _load_last():
    try:
        return open(CFG, encoding="utf-8").read().strip()
    except OSError:
        return ""


def _save_last(port):
    try:
        open(CFG, "w", encoding="utf-8").write(port)
    except OSError:
        pass


def pick_port():
    """自动找蓝牙串口: 上次用过的直接沿用; 否则菜单选择 (描述带 Bluetooth/蓝牙 的优先)"""
    import serial.tools.list_ports as lp
    ports = list(lp.comports())
    if not ports:
        raise SystemExit("[!] 没找到任何串口, 请确认蓝牙模块已配对")
    bt = [p for p in ports
          if "bluetooth" in (p.description or "").lower()
          or "蓝牙" in (p.description or "")]
    cand = bt if bt else ports
    devices = [p.device for p in cand]
    last = _load_last()

    if len(cand) == 1:
        print("自动选择串口: %s (%s)" % (cand[0].device, cand[0].description))
        return cand[0].device
    if last in devices:
        print("沿用上次的串口: %s (删 %s 可重新选择)" % (last, CFG))
        return last

    print("找到多个串口, 请选择 HC-06 所在的口 (选一次以后就记住了):")
    for i, p in enumerate(cand, 1):
        mark = "   <- 上次用的" if p.device == last else ""
        print("  %d) %-8s %s%s" % (i, p.device, p.description, mark))
    while True:
        s = input("输入序号 (回车=1): ").strip()
        if not s:
            pick = cand[0].device
        elif s.isdigit() and 1 <= int(s) <= len(cand):
            pick = cand[int(s) - 1].device
        else:
            print("输入无效, 重来")
            continue
        _save_last(pick)
        return pick


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    port = args[0] if args else pick_port()
    baud = int(sys.argv[sys.argv.index("--baud") + 1]) if "--baud" in sys.argv else 9600

    ser = serial.Serial(port, baud, timeout=0.05)
    time.sleep(0.3)

    speed = 40
    cur_frame = frame(CMD["x"], 0)     # 当前指令帧 (初始为停止)
    last_sent = 0.0
    print("已连 %s @ %d" % (port, baud))
    print("w/s/a/d 控制(可组合边走边转) | x 停止 | +/- 力度(当前 %d) | q 退出" % speed)
    print("按住方向键 = 持续走; 松开 0.5 秒内车会自动停 (固件的断联保护)")
    print()

    prev_q = prev_plus = prev_minus = False
    while True:
        now = time.time()
        q = key_down("q")
        plus = key_down("plus") or key_down("numpad_plus")
        minus = key_down("minus") or key_down("numpad_minus")

        # q / +/- 用"按下瞬间"触发一次, 不受长按连发影响
        if q and not prev_q:
            break
        if plus and not prev_plus:
            speed = min(100, speed + 10)
            print("力度 -> %d" % speed)
        if minus and not prev_minus:
            speed = max(10, speed - 10)
            print("力度 -> %d" % speed)
        prev_q, prev_plus, prev_minus = q, plus, minus

        # 前后 / 左右 独立读取 -> 混控帧, 支持边走边转
        lin = 0
        rot = 0
        if not (key_down("x") or key_down("space")):
            if key_down("w") or key_down("up"):     lin = speed
            elif key_down("s") or key_down("down"): lin = -speed
            if key_down("d") or key_down("right"):  rot = speed
            elif key_down("a") or key_down("left"): rot = -speed

        f = frame(CMD["x"], 0) if (lin == 0 and rot == 0) else mix_frame(lin, rot)
        if f != cur_frame:
            cur_frame = f
            print("%-10s 力度=%d" % (describe(lin, rot), speed))

        # 150ms 重发当前指令 (松手发的是停止帧, 车上断联保护不会触发)
        if now - last_sent >= RESEND:
            ser.write(cur_frame)
            ser.flush()
            last_sent = now

        time.sleep(0.03)

    send(ser, CMD["x"], 0)
    ser.close()
    print("已停止, 退出")


if __name__ == "__main__":
    try:
        main()
    except SystemExit as e:
        if e.code:
            print(e.code if isinstance(e.code, str) else __doc__)
    except Exception as e:
        print("[!] 出错: %s" % e)
        print("    常见原因: 蓝牙被别的程序占用 / 模块没配对 / 选错串口")
    input("\n按回车关闭窗口...")
