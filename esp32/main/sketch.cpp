// Standalone gamepad -> ESP32 -> existing STM32 mode 7 bridge.
#include <Arduino.h>
#include <Bluepad32.h>
#include "car_bridge.h"
static HardwareSerial carUart(2);
static ControllerPtr gamepad = nullptr;
static bridge_state_t state = {};
static uint32_t nextTx, nextLog, reportCount;
static bool wasMoving;
static uint32_t bootSince;
static bool bootTracking, bootHandled;
static const uint8_t stopFrame[] = {0xAA, 0x55, 0x05, 0x00, 0xFA};
static void stopNow() {
    carUart.write(stopFrame, sizeof(stopFrame));
    wasMoving = false;
}
static void onConnected(ControllerPtr ctl) {
    // Bluepad32 can notify connection before the first report sets its class.
    if (gamepad) { ctl->disconnect(); return; }
    gamepad = ctl;
    bridge_reset(&state, true);
    stopNow();
    auto p = ctl->getProperties();
    String model = ctl->getModelName();
    Console.printf("CONNECTED model=%s VID=%04x PID=%04x; release RT/LT, center left stick\n",
                   model.c_str(), p.vendor_id, p.product_id);
    BP32.enableNewBluetoothConnections(false);
}
static void onDisconnected(ControllerPtr ctl) {
    if (ctl != gamepad) return;
    gamepad = nullptr;
    bridge_reset(&state, false);
    stopNow();
    Console.println("DISCONNECTED: STOP sent");
    BP32.enableNewBluetoothConnections(true);
}
void setup() {
    // UART0 belongs to USB Console. Car uses UART2, D17 TX only.
    carUart.begin(9600, SERIAL_8N1, 16, 17);
    pinMode(0, INPUT_PULLUP);
    stopNow();
    Console.printf("CAR BRIDGE v2 RT/LT + LEFT STEERING; %s; TX=D17, 9600 8N1\n", BP32.firmwareVersion());
    BP32.setup(onConnected, onDisconnected);
    BP32.enableVirtualDevice(false);
    BP32.enableBLEService(false);
    // Normal boots keep bonds for standalone reconnection.
    nextTx = nextLog = millis();
}
void loop() {
    uint32_t now = millis();
    // BOOT is a strap pin: use it AFTER startup, not held during power-on.
    if (digitalRead(0) == LOW) {
        if (!bootTracking) { bootSince = now; bootTracking = true; }
        if (!bootHandled && (uint32_t)(now - bootSince) >= 3000) {
            bootHandled = true;
            bridge_reset(&state, false);
            stopNow();
            if (gamepad) gamepad->disconnect();
            BP32.forgetBluetoothKeys();
            BP32.enableNewBluetoothConnections(true);
            Console.println("BOOT 3 seconds: stopped, pairing keys cleared");
        }
    } else { bootTracking = false; bootHandled = false; }
    bridge_gate_update(&state, now);
    bool updated = BP32.update();
    now = millis();
    if (!bootHandled && updated && gamepad && gamepad->isConnected() && gamepad->hasData()) {
        if (gamepad->isGamepad()) {
            bridge_report(&state, now, gamepad->throttle(), gamepad->brake(), gamepad->axisX(), gamepad->b());
            ++reportCount;
        } else {
            bridge_reset(&state, false);
            stopNow();
            gamepad->disconnect();
            Console.println("Non-gamepad input rejected after first report");
        }
    }
    bridge_command_t cmd = bridge_command(&state, now);
    bool moving = cmd.linear != 0 || cmd.rotation != 0;
    if (wasMoving && !moving) stopNow();
    if ((int32_t)(now - nextTx) >= 0) {
        nextTx = now + 20; // 50 Hz; no catch-up burst after a stall.
        uint8_t packet[6];
        bridge_mix_packet(cmd, packet);
        carUart.write(packet, sizeof(packet));
        wasMoving = moving;
    }
    if ((int32_t)(now - nextLog) >= 0) {
        nextLog = now + 500;
        Console.printf("LINK=%u ARM=%u RT=%d LT=%d LX=%d B=%u LIN=%d ROT=%d AGE=%lu REPORTS=%lu\n",
                       state.connected, state.armed, state.rt, state.lt, state.lx, state.emergency,
                       cmd.linear, cmd.rotation,
                       state.seen ? (unsigned long)(now - state.last_report) : 0UL,
                       (unsigned long)reportCount);
    }
    delay(1);
}
