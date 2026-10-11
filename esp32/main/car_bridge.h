#ifndef CAR_BRIDGE_H
#define CAR_BRIDGE_H
#include <stdint.h>
#define BRIDGE_DEADZONE 60
#define BRIDGE_STEER_LIMIT 50
// STM32 RC uses 60 cm/s at command 100; nearest command to 47 cm/s is 78 (46.8).
#define BRIDGE_SPEED_LIMIT 78
#define BRIDGE_TRIGGER_DEADZONE 30
// This X-Pro's full trigger report was measured at 1020, not 1023.
#define BRIDGE_TRIGGER_MAX 1020
#define BRIDGE_ARM_MS 300u
typedef struct {
    uint32_t last_report, neutral_since;
    int16_t rt, lt, lx;
    uint8_t connected, seen, armed, neutral_tracking, emergency;
} bridge_state_t;
typedef struct { int8_t linear, rotation; } bridge_command_t;
static inline int bridge_abs(int n) { return n < 0 ? -n : n; }
static inline int8_t bridge_axis(int raw, int limit) {
    if (raw > 512) raw = 512;
    if (raw < -512) raw = -512;
    int magnitude = bridge_abs(raw);
    if (magnitude <= BRIDGE_DEADZONE) return 0;
    magnitude = (magnitude - BRIDGE_DEADZONE) * limit / (512 - BRIDGE_DEADZONE);
    return (int8_t)(raw < 0 ? -magnitude : magnitude);
}
static inline int bridge_pedal(int raw, int limit) {
    if (raw <= BRIDGE_TRIGGER_DEADZONE) return 0;
    if (raw > BRIDGE_TRIGGER_MAX) raw = BRIDGE_TRIGGER_MAX;
    return (raw - BRIDGE_TRIGGER_DEADZONE) * limit / (BRIDGE_TRIGGER_MAX - BRIDGE_TRIGGER_DEADZONE);
}
static inline int bridge_neutral(const bridge_state_t* s) {
    return !s->emergency && s->rt <= BRIDGE_TRIGGER_DEADZONE &&
           s->lt <= BRIDGE_TRIGGER_DEADZONE && bridge_abs(s->lx) <= BRIDGE_DEADZONE;
}
static inline void bridge_reset(bridge_state_t* s, int connected) {
    bridge_state_t cleared = {0};
    *s = cleared;
    s->connected = connected != 0;
}
static inline void bridge_gate_update(bridge_state_t* s, uint32_t now) {
    // HID reports are event-driven: held sticks need not produce new reports.
    // Radio loss is supervised by Bluetooth, not the age of a joystick change.
    if (!s->connected || !s->seen) {
        s->armed = 0;
        s->neutral_tracking = 0;
        return;
    }
    if (!s->armed) {
        if (bridge_neutral(s)) {
            if (!s->neutral_tracking) {
                s->neutral_tracking = 1;
                s->neutral_since = now;
            } else if ((uint32_t)(now - s->neutral_since) >= BRIDGE_ARM_MS) s->armed = 1;
        } else s->neutral_tracking = 0;
    }
}
static inline void bridge_report(bridge_state_t* s, uint32_t now,
                                 int rt, int lt, int lx, int emergency) {
    bridge_gate_update(s, now);
    s->rt = (int16_t)(rt < 0 ? 0 : rt > 1023 ? 1023 : rt);
    s->lt = (int16_t)(lt < 0 ? 0 : lt > 1023 ? 1023 : lt);
    s->lx = (int16_t)(lx < -512 ? -512 : lx > 512 ? 512 : lx);
    s->emergency = emergency != 0;
    s->last_report = now; s->seen = 1;
    if (emergency) {
        s->armed = 0; s->neutral_tracking = 0;
        return;
    }
    bridge_gate_update(s, now);
}
static inline bridge_command_t bridge_command(bridge_state_t* s, uint32_t now) {
    bridge_command_t cmd = {0, 0};
    bridge_gate_update(s, now);
    if (s->connected && s->seen && s->armed && !s->emergency) {
        cmd.linear = (int8_t)(bridge_pedal(s->rt, BRIDGE_SPEED_LIMIT) - bridge_pedal(s->lt, BRIDGE_SPEED_LIMIT));
        // Steering also works with zero throttle: STM32 uses its spin channel.
        cmd.rotation = bridge_axis(s->lx, BRIDGE_STEER_LIMIT);
        // Keep steering sign in reverse: STM32 mode 7 performs the same
        // inversion as the existing remote's mode 2, exactly once.
    }
    return cmd;
}
static inline void bridge_mix_packet(bridge_command_t cmd, uint8_t packet[6]) {
    // Car reserves AA 55 inside payload; protect a future 100% setting too.
    if (cmd.linear == -86 && cmd.rotation == 85) cmd.rotation = 84;
    packet[0] = 0xAA; packet[1] = 0x55; packet[2] = 0x06;
    packet[3] = (uint8_t)cmd.linear; packet[4] = (uint8_t)cmd.rotation;
    packet[5] = packet[0] ^ packet[1] ^ packet[2] ^ packet[3] ^ packet[4];
}
#endif
