"""在电脑上接收小车蓝牙发来的数据 (验证模式四的链路, 不用开 VOFA+)。

用法:
    python bt_capture.py            先列出电脑上所有串口, 找到 HC-06 那个
    python bt_capture.py COM9       打开串口接收, 自动试 115200 和 9600

正常情况下应该看到每秒 20 行这样的数据 (目标速度, 实际速度):
    0.0,0.0
    0.0,0.0
按了 K5 启动后会变成:
    40.0,38.6
    40.0,39.2
两个波特率都收不到就是蓝牙没连上, 或者小车没进模式四。
"""
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("缺少 pyserial, 先执行: pip install pyserial")


def list_all():
    print("电脑上的串口:")
    ports = list(list_ports.comports())
    for p in ports:
        print(f"    {p.device:<8} {p.description}")
    if not ports:
        print("    (一个都没有)")
    print()
    print("HC-06 配对成功后, 一般会多出一个 'HC-06 上的标准串行' 之类的端口,")
    print("那个就是要用的。注意 Windows 会给蓝牙串口建两个端口, 用**传出/出站**那个。")
    print()
    print("然后运行: python bt_capture.py COMx")


def main():
    if len(sys.argv) < 2:
        list_all()
        return 1

    port = sys.argv[1]
    for baud in (115200, 9600):
        print(f"--- 试着用 {baud} 打开 {port} (5 秒) ---")
        try:
            ser = serial.Serial(port, baud, timeout=0.5)
        except serial.SerialException as e:
            print(f"打不开 {port}: {e}")
            return 1

        got = 0
        t0 = time.time()
        try:
            while time.time() - t0 < 5.0:
                line = ser.readline()
                if line:
                    got += 1
                    text = line.decode("utf-8", "replace").rstrip()
                    if got <= 3:
                        print(f"    [{time.time() - t0:4.1f}s] {text}")
        except KeyboardInterrupt:
            ser.close()
            print("\n已停止")
            return 0
        ser.close()

        if got > 0:
            print(f"\n>>> {baud} 收到 {got} 行数据, 链路通了!")
            print(f">>> 在 VOFA+ 里把波特率设成 {baud}, 数据引擎选 FireWater 就能看曲线。")
            print("\n继续接收中 (Ctrl+C 停止):")
            ser = serial.Serial(port, baud, timeout=0.5)
            n = 0
            try:
                while True:
                    line = ser.readline()
                    if line:
                        n += 1
                        print(f"{n:6d}  {line.decode('utf-8', 'replace').rstrip()}")
            except KeyboardInterrupt:
                print("\n已停止")
            finally:
                ser.close()
            return 0
        print("    没有数据\n")

    print("两种波特率都没收到数据。检查:")
    print("  1. 小车是不是在模式四 (OLED 上显示 '4 速度曲线')?")
    print("  2. 蓝牙是不是真连上了 (设备管理器里能看到 HC-06 的端口)?")
    print("  3. 打开的是不是'传出'那个端口?")
    print("  4. 是不是连着下载器/串口助手, 把端口占住了?")
    return 2


if __name__ == "__main__":
    sys.exit(main())
