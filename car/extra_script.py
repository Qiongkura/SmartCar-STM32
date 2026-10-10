"""让 pio run 在链接完 .elf 之后自动多生成一个 firmware.hex。

普中 ISP 这类串口下载工具只认 Intel HEX, PlatformIO 默认只出 .bin, 所以补这一步。
"""
Import("env")

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-O", "ihex",
        "$BUILD_DIR/${PROGNAME}.elf",
        "$BUILD_DIR/${PROGNAME}.hex",
    ]), "Building $BUILD_DIR/${PROGNAME}.hex"),
)
