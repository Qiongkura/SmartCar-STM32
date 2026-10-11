// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

#include <stddef.h>

// BTstack related
#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <gap.h>
#include <hci.h>
#include <btstack_event.h>
#include <stdio.h>

// Bluepad32 related
#include <arduino_platform.h>
#include <uni.h>

static btstack_packet_callback_registration_t car_link_events;
static void car_link_event(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    (void)channel; (void)size;
    if (packet_type != HCI_EVENT_PACKET || hci_event_packet_get_type(packet) != HCI_EVENT_LE_META) return;
    uint8_t sub = hci_event_le_meta_get_subevent_code(packet);
    if (sub == HCI_SUBEVENT_LE_CONNECTION_COMPLETE && hci_subevent_le_connection_complete_get_status(packet) == 0) {
        printf("CAR_BLE supervision=%u ms\n", hci_subevent_le_connection_complete_get_supervision_timeout(packet) * 10u);
    } else if (sub == HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE && hci_subevent_le_connection_update_complete_get_status(packet) == 0) {
        uint16_t timeout = hci_subevent_le_connection_update_complete_get_supervision_timeout(packet);
        printf("CAR_BLE updated supervision=%u ms\n", timeout * 10u);
        if (timeout > 100u) {
            gap_update_connection_parameters(hci_subevent_le_connection_update_complete_get_connection_handle(packet), 8, 24, 0, 72);
        }
    }
}

//
// Autostart
//
#if CONFIG_AUTOSTART_ARDUINO
void initBluepad32() {
#else
int app_main(void) {
#endif  // !CONFIG_AUTOSTART_ARDUINO
    // hci_dump_open(NULL, HCI_DUMP_STDOUT);

// Don't use BTstack buffered UART. It conflicts with the console.
#ifndef CONFIG_ESP_CONSOLE_UART_NONE
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    btstack_stdio_init();
#endif  // CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
#endif  // CONFIG_ESP_CONSOLE_UART_NONE

    // Configure BTstack for ESP32 VHCI Controller
    btstack_init();

    // hci_dump_init(hci_dump_embedded_stdout_get_instance());

    // Must be called before uni_init()
    uni_platform_set_custom(get_arduino_platform());

    // Init Bluepad32.
    uni_init(0 /* argc */, NULL /* argv */);

    // Configure on CPU0 before running the Bluetooth event loop.
    // BLE hardware exchanges link-layer packets even when HID values don't change.
    gap_set_link_supervision_timeout(1600); // Classic: 1 second, 0.625 ms units.
    gap_set_connection_parameters(0x60, 0x30, 8, 24, 0, 72, 0, 0); // BLE: 720 ms.
    car_link_events.callback = car_link_event;
    hci_add_event_handler(&car_link_events);

    // Does not return.
    btstack_run_loop_execute();
#if !CONFIG_AUTOSTART_ARDUINO
    return 0;
#endif  // !CONFIG_AUTOSTART_ARDUINO
}
