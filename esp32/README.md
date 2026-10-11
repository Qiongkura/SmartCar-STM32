# ESP32 蓝牙手柄接收器

让 **蓝牙游戏手柄直连小车**：手柄 → 蓝牙 → ESP32 → 串口 → STM32 小车模式七。

基于 [Bluepad32](https://bluepad32.readthedocs.io/) 蓝牙手柄库，支持 Switch Pro / PS / Xbox / 8BitDo 等主流手柄（完整列表见 [Bluepad32 支持列表](https://bluepad32.readthedocs.io/en/latest/supported_gamepads/)）。已实测 8BitDo Ultimate（BLE，VID=2DC8 PID=901B）。

运行时不需要电脑——ESP32 相当于一个"手柄版遥控器"，输出与 `remote/` 遥控器完全相同的 `AA 55 06` 协议帧（20ms/帧 @ 9600 8N1），直接插在小车原 HC-06 插口上。

## 功能

- **RT/LT 扳机油门**：按压深度决定前进/倒车目标速度，扳机松开停车
- **左摇杆转向**：RT/LT 松开时左摇杆可原地转向；转向曲线与原遥控器一致
- **安全保护**：首次回中保护（连接后必须回中一次才使能）、B 键紧急停车并取消使能、手柄断联自动发停车帧、ESP32 失效时小车端串口超时保护兜底
- **长按 BOOT 3 秒**清除蓝牙配对键，重新进入配对模式

## 接线

| ESP32 | 小车端 |
| --- | --- |
| D17（GPIO17，TX） | PA10 / 原 HC-06 插口 RXD |
| GND | 小车 GND |
| VIN | 小车稳压后 5V（调试期先用 USB 供电，只接 D17 + 共地） |

⚠️ 必须先拔掉 HC-06：HC-06 的 TXD 和 ESP32 的 D17 不能同时接 PA10。详细接线、上电顺序和首次配对步骤见 [接线与使用.md](接线与使用.md)。

## 构建与烧录

```
cd esp32
pio run -e esp32dev -t upload        # PlatformIO（ESP-IDF 框架）
```

如果 PlatformIO 与模板的 SCons 版本冲突，用裁剪构建脚本（只编 Arduino 核心，省掉未用的 WiFi/云服务，显著加快编译）：

```
powershell -File tools/build.ps1     # 编译 + 烧录
python tools/monitor.py              # 看日志（USB 串口 115200）
```

工程文件说明：

| 文件 | 作用 |
| --- | --- |
| `main/sketch.cpp` | 手柄连接管理、按键/扳机/摇杆 → 油门转向混控、安全逻辑 |
| `main/car_bridge.h` | 桥接状态机与 `AA 55` 协议帧打包（纯 C，可独立测试） |
| `main/main.c` | 模板自带的 ESP-IDF 启动入口（少量修改） |
| `platformio.ini` / `sdkconfig.defaults` | esp32dev 目标 + 最小化裁剪配置 |
| `tools/` | 裁剪构建、烧录校验、串口监视脚本 |

## 许可

模板与 Bluepad32 相关代码为 Apache-2.0（btstack 见其上游 LICENSE），本仓库的自定义桥接代码（`main/sketch.cpp`、`main/car_bridge.h`）同以 MIT 许可发布，见仓库根 LICENSE。

上游模板：https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template
