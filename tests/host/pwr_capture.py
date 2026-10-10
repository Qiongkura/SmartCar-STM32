"""通过 ST-Link 抓一段单片机供电电压的波形 (排查"电池供电时 3.3V 塌陷")

原理: 固件里有个供电波形记录器 (src/pwrlog.c), 用 ADC 反复测芯片内部的
1.20V 参考源, 反推出 VDD 随时间的变化, 存到 RAM 里。这个脚本用 OpenOCD
触发它、把数据导出来、换算成 mV 并画成图。

用法:
    python pwr_capture.py                  # 默认 2ms 一个点, 512 点 (约 1 秒)
    python pwr_capture.py 500              # 0.5ms 一个点
    python pwr_capture.py 10000            # 10ms 一个点 (约 5 秒, 看慢速抖动)
    python pwr_capture.py 2000 --aux       # 同时采 PA0/PA1 (要先接分压电阻!)

重要前提 (否则测出来的是假象):
    1. ST-Link **只接 SWCLK / SWDIO / GND**, 3.3V 那根线必须拔掉!
       插着 3.3V 的话是 ST-Link 在给板子供电, 整条可疑链路被绕开, 什么都测不出来。
    2. 板子用电池(12V)供电, 被测的负载(比如蓝牙模块)要插着。
    3. 如果拔掉 3.3V 后 OpenOCD 报 "Target voltage: 0.000" 连不上, 在
       ST-Link 的 3.3V 线和板子 3.3V 之间串一个 1kΩ 电阻 —— 这样它只
       传感电压(不耗电), 不会给板子供电。
"""
import os
import re
import struct
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
ELF = os.path.join(PROJ, ".pio", "build", "bluepill_f103c8", "firmware.elf")
BIN = os.path.join(PROJ, ".pio", "build", "bluepill_f103c8", "pwr_vref.bin")
BIN_A0 = os.path.join(PROJ, ".pio", "build", "bluepill_f103c8", "pwr_a0.bin")
BIN_A1 = os.path.join(PROJ, ".pio", "build", "bluepill_f103c8", "pwr_a1.bin")

NEED = ["pwrlog_req", "pwrlog_done", "pwrlog_period_us", "pwrlog_n", "pwrlog_aux_en",
        "pwrlog_vref", "pwrlog_aux0", "pwrlog_aux1", "pwrlog_count",
        "pwrlog_elapsed_ms", "pwrlog_pvd_low", "pwrlog_lo_mv", "pwrlog_hi_mv",
        "pwrlog_csr_before", "pwrlog_csr_after", "vdd_now"]


def find_tool(name):
    """在 PlatformIO 的包目录里找 arm-none-eabi-nm / openocd"""
    home = os.path.expanduser("~")
    pkgs = os.path.join(home, ".platformio", "packages")
    for d in os.listdir(pkgs):
        for sub in ("bin", ""):
            p = os.path.join(pkgs, d, sub, name)
            if os.path.isfile(p):
                return p
    raise SystemExit("找不到 %s, 请确认 PlatformIO 装好了" % name)


def symbols():
    nm = find_tool("arm-none-eabi-nm.exe" if os.name == "nt" else "arm-none-eabi-nm")
    out = subprocess.run([nm, ELF], capture_output=True, text=True).stdout
    sym = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3:
            sym[parts[2]] = int(parts[0], 16)
    missing = [n for n in NEED if n not in sym]
    if missing:
        raise SystemExit("固件里没有这些符号: %s\n(是不是还没编译?)" % missing)
    return sym


def run_openocd(sym, period_us, n, aux, wait_ms):
    ocd = find_tool("openocd.exe" if os.name == "nt" else "openocd")
    scfg = os.path.join(os.path.dirname(ocd), "..", "scripts")
    cmds = [
        "init",
        "mww 0x%08X %d" % (sym["pwrlog_period_us"], period_us),
        "mwh 0x%08X %d" % (sym["pwrlog_n"], n),
        "mwb 0x%08X %d" % (sym["pwrlog_aux_en"], 1 if aux else 0),
        "mwb 0x%08X 0" % sym["pwrlog_done"],
        "mwb 0x%08X 1" % sym["pwrlog_req"],          # 触发
        "sleep %d" % wait_ms,
        # 每条读取前先 echo 一个标签, 这样解析时不用猜顺序
        "echo R:DONE",   "mdb 0x%08X 1" % sym["pwrlog_done"],
        "echo R:COUNT",  "mdh 0x%08X 1" % sym["pwrlog_count"],
        "echo R:ELAPSED", "mdw 0x%08X 1" % sym["pwrlog_elapsed_ms"],
        "echo R:PVDLOW", "mdw 0x%08X 1" % sym["pwrlog_pvd_low"],
        "echo R:LOMV",   "mdw 0x%08X 1" % sym["pwrlog_lo_mv"],
        "echo R:HIMV",   "mdw 0x%08X 1" % sym["pwrlog_hi_mv"],
        "echo R:CSRB",   "mdw 0x%08X 1" % sym["pwrlog_csr_before"],
        "echo R:CSRA",   "mdw 0x%08X 1" % sym["pwrlog_csr_after"],
        "echo R:VDDNOW", "mdw 0x%08X 1" % sym["vdd_now"],
        "dump_image {%s} 0x%08X %d" % (BIN.replace("\\", "/"), sym["pwrlog_vref"], n * 2),
    ]
    if aux:
        cmds.append("dump_image {%s} 0x%08X %d" % (BIN_A0.replace("\\", "/"), sym["pwrlog_aux0"], n * 2))
        cmds.append("dump_image {%s} 0x%08X %d" % (BIN_A1.replace("\\", "/"), sym["pwrlog_aux1"], n * 2))
    cmds.append("shutdown")

    cmd = [ocd, "-s", scfg, "-f", "interface/stlink.cfg", "-f", "target/stm32f1x.cfg"]
    for c in cmds:
        cmd += ["-c", c]
    r = subprocess.run(cmd, capture_output=True, text=True)
    txt = r.stdout + r.stderr
    if "Target voltage: 0" in txt:
        print("!! ST-Link 读到的目标电压是 0 —— 检查 3.3V 传感线/地线是否接好")
    return txt


def parse(txt):
    """按 "R:XXX" 标签把 OpenOCD 打印的寄存器值抓出来
       (mdb/mdh/mdw 的输出格式都是 "0x20000000: 01" 一行一个)"""
    val = {}
    label = None
    for line in txt.splitlines():
        m = re.match(r"^R:(\w+)\s*$", line.strip())
        if m:
            label = m.group(1)
            continue
        m = re.match(r"^0x[0-9a-fA-F]+: ([0-9a-fA-F]+)\s*$", line.strip())
        if m and label:
            val[label] = int(m.group(1), 16)
            label = None
    return val


# RCC_CSR 里的复位标志位 (STM32F1 参考手册)
CSR_FLAGS = [(31, "LPWRRSTF"), (30, "WWDGRSTF"), (29, "IWDGRSTF"),
             (28, "SFTRSTF"), (27, "PORRSTF"), (26, "PINRSTF")]
# 掉电复位(POR)和引脚复位(PIN)这两个才算"芯片被拉停过";
# 另外三个分别是低功耗/看门狗/软件复位, 和我们排查的供电问题无关。
BROWNOUT_MASK = (1 << 27) | (1 << 26)


def csr_str(v):
    names = [n for b, n in CSR_FLAGS if v & (1 << b)]
    return ",".join(names) if names else "无"


def plot(mv, pvd, period_us):
    n = len(mv)
    cols = 96
    per = max(1, n // cols)
    print()
    print("  电压波形 (每列 %d 个采样点, 共 %.2f 秒):" % (per, n * period_us / 1e6))
    lo, hi = min(mv), max(mv)
    span = max(1.0, hi - lo)
    for row in range(16, -1, -1):
        thr = lo + span * row / 16.0
        line = "".join("#" if mv[i] >= thr else " " for i in range(0, n, per))
        print("  %5.0f |%s" % (thr, line))
    print("        +" + "-" * ((n + per - 1) // per))
    if any(pvd):
        marks = "".join("^" if pvd[i] else " " for i in range(0, n, per))
        print("   跌破2.9V %s" % marks)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    aux = "--aux" in sys.argv
    period_us = int(args[0]) if args else 2000
    n = 512
    wait_ms = int(n * period_us / 1000) + 1500

    sym = symbols()
    print("触发记录: 间隔 %dus, %d 点, 约 %.2f 秒 ..." % (period_us, n, n * period_us / 1e6))
    txt = run_openocd(sym, period_us, n, aux, wait_ms)
    v = parse(txt)
    if "DONE" not in v:
        print("没读到数据 —— ST-Link 没插好, 或者固件还是旧的(没有 pwrlog 模块)。")
        print("OpenOCD 原始输出的末尾:")
        print(txt[-1500:])
        return

    done, count, elapsed = v["DONE"], v.get("COUNT", 0), v.get("ELAPSED", 0)
    pvd_low, lo_mv, hi_mv = v.get("PVDLOW", 0), v.get("LOMV", 0), v.get("HIMV", 0)
    csr_b, csr_a = v.get("CSRB", 0), v.get("CSRA", 0)
    vdd_now = struct.unpack("<f", struct.pack("<I", v.get("VDDNOW", 0)))[0]

    print()
    print("记录完成 = %d   点数 = %d   实际耗时 = %dms" % (done, count, elapsed))
    print("最低电压 = %d mV   最高电压 = %d mV   (芯片自测, ±3%%)" % (lo_mv, hi_mv))
    print("当前实时读数 vdd_now = %.2f V" % vdd_now)
    print("PVD(2.9V) 报警点数 = %d / %d" % (pvd_low, count))
    print("复位标志: 记录前 [%s]  记录后 [%s]  (固件在开始记录时清过一次)"
          % (csr_str(csr_b), csr_str(csr_a)))
    if csr_a & BROWNOUT_MASK:
        print("  !! 记录期间芯片被复位过 (PORRSTF/PINRSTF) —— 供电确实把芯片拉停过")
    if csr_b & BROWNOUT_MASK:
        print("  (记录开始前就有掉电复位标志: 上电以来至少被拉停过一次)")
    if done != 1:
        print("  !! 记录没有跑完 —— 中途复位了, 基本可以确定是供电问题")

    if not os.path.isfile(BIN):
        print("没有导出到波形文件")
        return
    raw = open(BIN, "rb").read()
    vals = struct.unpack("<%dH" % (len(raw) // 2), raw)
    mv = [1200.0 * 4096.0 / (v & 0x0FFF) if (v & 0x0FFF) else 0.0 for v in vals]
    pvd = [(v & 0x8000) != 0 for v in vals]
    plot(mv, pvd, period_us)

    if aux:
        for f, name, r1, r2 in [(BIN_A0, "PA0", 10000.0, 1000.0),
                                (BIN_A1, "PA1", 10000.0, 10000.0)]:
            if not os.path.isfile(f):
                continue
            raw = open(f, "rb").read()
            a = struct.unpack("<%dH" % (len(raw) // 2), raw)
            print()
            print("%s 通道 (接的是 %s 分压):" % (name, "12V(10k:1k)" if name == "PA0" else "5V(10k:10k)"))
            for i in range(0, len(a), max(1, len(a) // 16)):
                vdd_i = mv[i] / 1000.0
                volts = (a[i] / 4096.0) * vdd_i * (r1 + r2) / r2
                print("    t=%4dms  raw=%4d  -> %.2f V" % (i * period_us // 1000, a[i], volts))


if __name__ == "__main__":
    main()
